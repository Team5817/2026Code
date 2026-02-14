package com.team5817.frc2026.planners;

import com.team5817.frc2026.subsystems.Shooter.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import java.util.EnumMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShootingPlanner {

  private static final int CONVERGENCE_ITERS = 5;
  private static final double MIN_NORM = 1e-4;

  private final Map<ShootingTarget, DoubleSupplier> hoodAngleSuppliers = new EnumMap<>(ShootingTarget.class);
  private final Map<ShootingTarget, DoubleSupplier> turretAngleSuppliers = new EnumMap<>(ShootingTarget.class);
  private final Map<ShootingTarget, DoubleSupplier> flywheelSpeedSuppliers = new EnumMap<>(ShootingTarget.class);

  private final Supplier<Pose2d> shooterPoseSupplier;
  private final Supplier<ChassisSpeeds> shooterVelocitySupplier;
  private BooleanSupplier atStateSupplier;
  private DoubleSupplier timeSinceVision;

  private final DoubleUnaryOperator timeForDistance;
  private final ShootingConfig config;

  private ShootingPlanner(Builder builder) {
    this.shooterPoseSupplier = () -> builder.shooterPoseSupplier.get().wpi();
    this.shooterVelocitySupplier = () -> builder.shooterVelocitySupplier.get().wpi();
    this.atStateSupplier = builder.atStateSupplier;
    this.timeSinceVision = builder.timeSinceVision != null ? builder.timeSinceVision : () -> Double.POSITIVE_INFINITY; // default if not set
    this.config = ShootingConfig.defaultConfig();

    this.timeForDistance =
        d -> {
          Double v = (Double) config.timeMap.get(d);
          return (v != null && Double.isFinite(v)) ? v : 0.0;
        };

    // Setup future-to-target suppliers
    Map<ShootingTarget, Supplier<Translation2d>> futureTo = new EnumMap<>(ShootingTarget.class);

    futureTo.put(ShootingTarget.HUB, () -> computeFutureVector(ShootingTarget.HUB));
    futureTo.put(ShootingTarget.LOBL, () -> computeFutureVector(ShootingTarget.LOBL));
    futureTo.put(ShootingTarget.LOBR, () -> computeFutureVector(ShootingTarget.LOBR));
    futureTo.put(
        ShootingTarget.LOB,
        () -> {
          Translation2d l = computeFutureVector(ShootingTarget.LOBL);
          Translation2d r = computeFutureVector(ShootingTarget.LOBR);
          return l.getNorm() < r.getNorm() ? l : r;
        });

    // Output suppliers (NaN-safe)
    for (ShootingTarget t : ShootingTarget.values()) {
      Supplier<Translation2d> vec = futureTo.get(t);

      hoodAngleSuppliers.put(
          t,
          () -> {
            double d = vec.get().getNorm();
            if (!Double.isFinite(d) || d < MIN_NORM) return 0.0;

            Double val = t.getHoodMap().get(d);
            return val != null && Double.isFinite(val) ? val : 0.0;
          });

      flywheelSpeedSuppliers.put(
          t,
          () -> {
            double d = vec.get().getNorm();
            if (!Double.isFinite(d) || d < MIN_NORM) return 0.0;

            Double val = t.getFlywheelMap().get(d);
            return val != null && Double.isFinite(val) ? val : 0.0;
          });

      turretAngleSuppliers.put(
          t,
          () -> {
            Translation2d v = vec.get();
            double x = v.getX();
            double y = v.getY();

            if (!Double.isFinite(x) || !Double.isFinite(y)) return 0.0;
            if (Math.hypot(x, y) < MIN_NORM) return 0.0;

            return Math.toDegrees(Math.atan2(y, x));
          });
    }
  }

  public static Builder builder() {
    return new Builder();
  }

  public static class Builder {
    private Supplier<com.team254.lib.geometry.Pose2d> shooterPoseSupplier;
    private Supplier<com.team254.lib.swerve.ChassisSpeeds> shooterVelocitySupplier;
    private BooleanSupplier atStateSupplier;
    private DoubleSupplier timeSinceVision;

    public Builder shooterPoseSupplier(Supplier<com.team254.lib.geometry.Pose2d> shooterPoseSupplier) {
      this.shooterPoseSupplier = shooterPoseSupplier;
      return this;
    }

    public Builder shooterVelocitySupplier(Supplier<com.team254.lib.swerve.ChassisSpeeds> shooterVelocitySupplier) {
      this.shooterVelocitySupplier = shooterVelocitySupplier;
      return this;
    }

    public Builder atStateSupplier(BooleanSupplier atStateSupplier) {
      this.atStateSupplier = atStateSupplier;
      return this;
    }

    public Builder timeSinceVisionSupplier(DoubleSupplier timeSinceVision) {
      this.timeSinceVision = timeSinceVision;
      return this;
    }

    public ShootingPlanner build() {
      if (shooterPoseSupplier == null) {
        throw new IllegalStateException("shooterPoseSupplier must be set");
      }
      if (shooterVelocitySupplier == null) {
        throw new IllegalStateException("shooterVelocitySupplier must be set");
      }
      if (atStateSupplier == null) {
        throw new IllegalStateException("atStateSupplier must be set");
      }
      // timeSinceVision is optional; default is positive infinity

      return new ShootingPlanner(this);
    }
  }

  /* Setter for updating vision timing supplier after construction */
  public void setTimeSinceVisionSupplier(DoubleSupplier timeSinceVision) {
    this.timeSinceVision = timeSinceVision;
  }

  /* -------------------------------------------------------------------------- */
  /*                                Core Math                                   */
  /* -------------------------------------------------------------------------- */

  private Translation2d computeFutureVector(ShootingTarget target) {
    Pose2d pose = shooterPoseSupplier.get();
    ChassisSpeeds speeds = shooterVelocitySupplier.get();

    if (pose == null || speeds == null) {
      return new Translation2d(MIN_NORM, 0.0);
    }

    Pose2d futurePose = pose;
    Translation2d targetPos = target.getLocation().wpi();

    double distance = targetPos.minus(futurePose.getTranslation()).getNorm();

    for (int i = 0; i < CONVERGENCE_ITERS; i++) {
      double tof = timeForDistance.applyAsDouble(distance);
      if (!Double.isFinite(tof) || tof <= 0.0) break;

      double vx = Double.isFinite(speeds.vxMetersPerSecond) ? speeds.vxMetersPerSecond : 0.0;
      double vy = Double.isFinite(speeds.vyMetersPerSecond) ? speeds.vyMetersPerSecond : 0.0;
      double omega =
          Double.isFinite(speeds.omegaRadiansPerSecond) ? speeds.omegaRadiansPerSecond : 0.0;

      Twist2d twist = new Twist2d(vx * tof, vy * tof, omega * tof);

      futurePose = futurePose.exp(twist);

      distance = targetPos.minus(futurePose.getTranslation()).getNorm();
    }

    Translation2d result = targetPos.minus(futurePose.getTranslation());

    if (!Double.isFinite(result.getX())
        || !Double.isFinite(result.getY())
        || result.getNorm() < MIN_NORM) {
      return new Translation2d(MIN_NORM, 0.0);
    }

    return result;
  }

  /* -------------------------------------------------------------------------- */

  @AutoLogOutput(key = "Shooter/Planner/RecommendedState")
  public Shooter.State recommendedShooterState() {
    Pose2d current = shooterPoseSupplier.get();
    ChassisSpeeds speeds = shooterVelocitySupplier.get();

    if (current == null || speeds == null) {
      return Shooter.State.STOW;
    }

    Translation2d toHub = ShootingTarget.HUB.getLocation().wpi().minus(current.getTranslation());

    double tof = timeForDistance.applyAsDouble(toHub.getNorm());

    Twist2d twist =
        new Twist2d(
            speeds.vxMetersPerSecond * tof,
            speeds.vyMetersPerSecond * tof,
            speeds.omegaRadiansPerSecond * tof);

    Pose2d futureHub = current.exp(twist);

    Logger.recordOutput("Shooter/Planner/FuturePose", futureHub);

    com.team254.lib.geometry.Translation2d pos = new com.team254.lib.geometry.Translation2d(current.getTranslation());

    if (pos.inBounds(config.dangerBounds)) return Shooter.State.STOW;
    if (pos.inBounds(config.dangerBoundsFlipped)) return Shooter.State.STOW;
    if (pos.inBounds(config.dangerBoundsFlippedOpponent)) return Shooter.State.STOW;
    if (pos.inBounds(config.dangerBoundsOpponent)) return Shooter.State.STOW;


    if (futureHub.getTranslation().getX() < config.hubBounds.maxX()) return Shooter.State.HUB;

    return Shooter.State.LOB;
  }

  public Boolean shouldShoot() {
    if (!atStateSupplier.getAsBoolean()) return false;

    Shooter.State state = recommendedShooterState();
    ChassisSpeeds speeds = shooterVelocitySupplier.get();
    if (speeds == null) return false;

    double linearVel = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double angularVelDeg = Math.toDegrees(speeds.omegaRadiansPerSecond);

    ShootingTarget target =
        state == Shooter.State.HUB
            ? ShootingTarget.HUB
            : state == Shooter.State.LOB ? ShootingTarget.LOB : null;

    if (target == null) return false;
    if (linearVel > target.getVelocityThreshold()) return false;
    if (Math.abs(angularVelDeg) > target.getRotationThreshold()) return false;
    if (timeSinceVision.getAsDouble() > target.getTimeSinceVisionThreshold()) return false;
    if(state == Shooter.State.STOW) return false;
    if(new com.team254.lib.geometry.Translation2d(shooterPoseSupplier.get().getTranslation()).inBounds(config.blockedBounds)) return false;

    Logger.recordOutput("Shooter/Planner/time", Timer.getTimestamp());
    return true;
  }

  public DoubleSupplier getHoodAngleSupplier(ShootingTarget target) {
    return hoodAngleSuppliers.get(target);
  }


  public DoubleSupplier getTurretAngleSupplier(ShootingTarget target) {
    return turretAngleSuppliers.get(target);
  }

  public DoubleSupplier getFlywheelSpeedSupplier(ShootingTarget target) {
    return flywheelSpeedSuppliers.get(target);
  }
}
