package com.team5817.frc2026.planners;

import com.team5817.frc2026.ActiveTracker;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.*;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public final class ShootingPlanner {

  private static final int CONVERGENCE_ITERS = 10;
  private static final double LOOP_PERIOD_SECS = 0.02;

  private static ShotSolution cachedSolution;
  private static double lastSolveTimestamp = -1.0;

  private static final LinearFilter turretVelFilter = LinearFilter.movingAverage(5);
  private static final LinearFilter hoodVelFilter = LinearFilter.movingAverage(5);

  private static Rotation2d lastTurretAngle = null;
  private static double lastHoodAngle = Double.NaN;

  private static Supplier<Pose2d> shooterPoseSupplier;
  private static Supplier<ChassisSpeeds> shooterVelocitySupplier;
  private static BooleanSupplier atStateSupplier;
  private static DoubleSupplier timeSinceVision = () -> Double.POSITIVE_INFINITY;

  private static final ShootingConfig config = ShootingConfig.defaultConfig();

  private static final DoubleUnaryOperator timeForDistance =
      d -> {
        Double v = (Double) config.timeMap.get(d);
        return (v != null && Double.isFinite(v)) ? v : 0.0;
      };

  private record ShotSolution(
      double turretAngleDeg,
      double turretVelocityFFDegPerSec,
      double hoodAngleDeg,
      double hoodVelocityFFDegPerSec,
      double flywheelSpeed) {}

  private ShootingPlanner() {}

  public static void configure(
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> velocitySupplier,
      BooleanSupplier atState,
      DoubleSupplier visionAge) {

    shooterPoseSupplier = poseSupplier;
    shooterVelocitySupplier = velocitySupplier;
    atStateSupplier = atState;
    if (visionAge != null) timeSinceVision = visionAge;
  }

  /* ======================== SOLVER ======================== */

  private static ShotSolution solve(ShootingTarget target) {
    Pose2d pose = shooterPoseSupplier.get();
    ChassisSpeeds speeds = shooterVelocitySupplier.get();

    if (pose == null || speeds == null) {
      Logger.recordOutput("ShootingPlanner/Error", "Null pose or velocity");
      return null;
    }

    Translation2d targetPos = target.getLocation().wpi();
    Pose2d turretPose = pose.transformBy(config.robotToTurret);

    double robotYaw = pose.getRotation().getRadians();

    double turretVx =
        speeds.vxMetersPerSecond
            + speeds.omegaRadiansPerSecond
                * (config.robotToTurret.getY() * Math.cos(robotYaw)
                    - config.robotToTurret.getX() * Math.sin(robotYaw));

    double turretVy =
        speeds.vyMetersPerSecond
            + speeds.omegaRadiansPerSecond
                * (config.robotToTurret.getX() * Math.cos(robotYaw)
                    - config.robotToTurret.getY() * Math.sin(robotYaw));

    Pose2d lookaheadPose = turretPose;
    double distance = targetPos.getDistance(turretPose.getTranslation());

    for (int i = 0; i < CONVERGENCE_ITERS; i++) {
      double tof = timeForDistance.applyAsDouble(distance);
      if (tof <= 0.0) break;

      Translation2d offset = new Translation2d(turretVx * tof, turretVy * tof);
      lookaheadPose =
          new Pose2d(
              turretPose.getTranslation().plus(offset),
              turretPose.getRotation());

      distance = targetPos.getDistance(lookaheadPose.getTranslation());
    }

    Translation2d toTarget = targetPos.minus(lookaheadPose.getTranslation());
    Rotation2d turretAngle = toTarget.getAngle();
    double turretAngleDeg = turretAngle.getDegrees();

    double hoodAngleDeg = target.getHoodMap().get(distance);
    double hoodAngleRad = Math.toRadians(hoodAngleDeg);

    double flywheelSpeed = target.getFlywheelMap().get(distance);

    /* ---------- Velocity FF ---------- */

    if (lastTurretAngle == null) {
      lastTurretAngle = turretAngle;
    }
    if (Double.isNaN(lastHoodAngle)) {
      lastHoodAngle = hoodAngleRad;
    }

    double turretVelDegPerSec =
        (turretAngle.getDegrees() - lastTurretAngle.getDegrees()) / LOOP_PERIOD_SECS;
    double hoodVelDegPerSec =
        (hoodAngleRad - lastHoodAngle) / LOOP_PERIOD_SECS * (180.0 / Math.PI);

    double turretFF = turretVelFilter.calculate(turretVelDegPerSec);
    double hoodFF = hoodVelFilter.calculate(hoodVelDegPerSec);

    lastTurretAngle = turretAngle;
    lastHoodAngle = hoodAngleRad;

    /* ---------- Logging ---------- */

    Logger.recordOutput("ShootingPlanner/TurretAngleDeg", turretAngleDeg);
    Logger.recordOutput("ShootingPlanner/HoodAngleDeg", hoodAngleDeg);
    Logger.recordOutput("ShootingPlanner/TurretVelDegPerSec", turretFF);
    Logger.recordOutput("ShootingPlanner/HoodVelDegPerSec", hoodFF);
    Logger.recordOutput("ShootingPlanner/FlywheelSpeed", flywheelSpeed);
    Logger.recordOutput("ShootingPlanner/LookaheadDistance", distance);
    Logger.recordOutput("ShootingPlanner/TurretVelocity",
        new ChassisSpeeds(turretVx, turretVy, 0.0));

    return new ShotSolution(
        turretAngleDeg,
        turretFF,
        hoodAngleDeg,
        hoodFF,
        flywheelSpeed);
  }

  private static ShotSolution getSolution(ShootingTarget target) {
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (cachedSolution == null || now != lastSolveTimestamp) {
      cachedSolution = solve(target);
      lastSolveTimestamp = now;
    }
    return cachedSolution;
  }

  /* ===================== Public API ===================== */

  public static double getTurretAngle(ShootingTarget target) {
    ShotSolution s = getSolution(target);
    return s != null ? s.turretAngleDeg : 0.0;
  }

  public static double getTurretVelocityFF(ShootingTarget target) {
    ShotSolution s = getSolution(target);
    return s != null ? s.turretVelocityFFDegPerSec : 0.0;
  }

  public static double getHoodAngle(ShootingTarget target) {
    ShotSolution s = getSolution(target);
    return s != null ? s.hoodAngleDeg : 0.0;
  }

  public static double getHoodVelocityFF(ShootingTarget target) {
    ShotSolution s = getSolution(target);
    return s != null ? s.hoodVelocityFFDegPerSec : 0.0;
  }

  public static double getFlywheelSpeed(ShootingTarget target) {
    ShotSolution s = getSolution(target);
    return s != null ? s.flywheelSpeed : 0.0;
  }

  /* ===================== Shoot Logic ===================== */

  @AutoLogOutput(key = "Shooter/Planner/RecommendedState")
  public static Shooter.State recommendedShooterState() {
    Pose2d current = shooterPoseSupplier.get();
    if (current == null) return Shooter.State.STOW;

    if (config.dangerBounds.contains(current.getTranslation()))
      return Shooter.State.STOW;
    if (current.getTranslation().getX() < config.hubBounds.maxX())
      return Shooter.State.HUB;

    return Shooter.State.LOB;
  }

  public static boolean shouldShoot() {
    if (!atStateSupplier.getAsBoolean()) return false;

    if (recommendedShooterState() == Shooter.State.HUB && ActiveTracker.isActive()) {
      return false;
    }

    ChassisSpeeds speeds = shooterVelocitySupplier.get();
    if (speeds == null) return false;

    if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        > config.maxShootVelocity) {
      return false;
    }

    if (Math.toDegrees(Math.abs(speeds.omegaRadiansPerSecond))
        > config.maxShootOmegaDeg) {
      return false;
    }

    return timeSinceVision.getAsDouble() <= config.maxVisionAge;
  }
}
