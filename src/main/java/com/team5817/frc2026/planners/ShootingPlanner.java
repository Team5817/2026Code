package com.team5817.frc2026.planners;

import com.team5817.frc2026.field.FieldConstants;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.lib.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.EnumMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShootingPlanner {

  private static final int CONVERGENCE_ITERS = 10; // og 5, increased for better drag convergence
  private static final double MIN_NORM = 1e-4;
  private static final double LOB_HYSTERESIS = 0.2; // meters


  private double dragConstantK = Double.POSITIVE_INFINITY; // TUNE THIS — start at 0.5s

  private final Map<ShootingTarget, DoubleSupplier> hoodAngleSuppliers =
      new EnumMap<>(ShootingTarget.class);
  private final Map<ShootingTarget, DoubleSupplier> turretAngleSuppliers =
      new EnumMap<>(ShootingTarget.class);
  private final Map<ShootingTarget, DoubleSupplier> flywheelSpeedSuppliers =
      new EnumMap<>(ShootingTarget.class);

  private final Supplier<Pose2d> shooterPoseSupplier;
  private final Supplier<ChassisSpeeds> shooterVelocitySupplier;
  private BooleanSupplier atStateSupplier;
  private DoubleSupplier timeSinceVision;

  private final DoubleUnaryOperator timeForDistance;
  private final ShootingConfig config;

  private ShootingTarget lastLobSide = ShootingTarget.LOBL;

  private ShootingPlanner(Builder builder) {
    this.shooterPoseSupplier = () -> builder.shooterPoseSupplier.get().wpi();
    this.shooterVelocitySupplier = () -> builder.shooterVelocitySupplier.get().wpi();
    this.atStateSupplier = builder.atStateSupplier;
    this.timeSinceVision =
        builder.timeSinceVision != null ? builder.timeSinceVision : () -> Double.POSITIVE_INFINITY;

    this.config = ShootingConfig.defaultConfig();

    this.timeForDistance =
        d -> {
          Double v = (Double) config.timeMap.get(d);
          return (v != null && Double.isFinite(v)) ? v : 0.0;
        };

    Map<ShootingTarget, Supplier<Translation2d>> futureTo = new EnumMap<>(ShootingTarget.class);

    futureTo.put(ShootingTarget.HUB, () -> computeFutureVector(ShootingTarget.HUB));
    futureTo.put(ShootingTarget.LOBL, () -> computeFutureVector(ShootingTarget.LOBL));
    futureTo.put(ShootingTarget.LOBR, () -> computeFutureVector(ShootingTarget.LOBR));

    futureTo.put(
        ShootingTarget.LOB,
        () -> {
          Pose2d pose = shooterPoseSupplier.get();
          if (pose == null) return new Translation2d(MIN_NORM, 0.0);

          Translation2d current = pose.getTranslation();
          Translation2d lNow = ShootingTarget.LOBL.getLocation().wpi().minus(current);
          Translation2d rNow = ShootingTarget.LOBR.getLocation().wpi().minus(current);

          double diff = lNow.getNorm() - rNow.getNorm();

          if (Math.abs(diff) > LOB_HYSTERESIS) {
            lastLobSide = diff < 0 ? ShootingTarget.LOBL : ShootingTarget.LOBR;
          }

          return lastLobSide == ShootingTarget.LOBL
              ? computeFutureVector(ShootingTarget.LOBL)
              : computeFutureVector(ShootingTarget.LOBR);
        });

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
            Logger.recordOutput(t + "FTG", v.getNorm());
            return v.getAngle().getDegrees();
          });
    }
  }

  /* -------------------------------------------------------------------------- */
  /*                         Drag Compensation Math                             */
  /* -------------------------------------------------------------------------- */

  private double dragAdjustedDisplacement(double velocity, double tof) {
    if (!Double.isFinite(dragConstantK) || dragConstantK <= 0.0) {
      // Drag disabled or invalid — fall back to Galilean
      return velocity * tof;
    }
    // Linear drag formula: v * k * (1 - e^(-tof/k))
    return velocity * dragConstantK * (1.0 - Math.exp(-tof / dragConstantK));
  }

  private double effectiveTof(double tof) {
    if (!Double.isFinite(dragConstantK) || dragConstantK <= 0.0) {
      return tof;
    }
    return dragConstantK * (1.0 - Math.exp(-tof / dragConstantK));
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

    double vx = Double.isFinite(speeds.vxMetersPerSecond) ? speeds.vxMetersPerSecond : 0.0;
    double vy = Double.isFinite(speeds.vyMetersPerSecond) ? speeds.vyMetersPerSecond : 0.0;
    double omega =
        Double.isFinite(speeds.omegaRadiansPerSecond) ? speeds.omegaRadiansPerSecond : 0.0;

    Pose2d futurePose = pose;
    Translation2d targetPos = target.getLocation().wpi();

    double distance = targetPos.minus(futurePose.getTranslation()).getNorm();

    for (int i = 0; i < CONVERGENCE_ITERS; i++) {
      double tof = timeForDistance.applyAsDouble(distance);
      if (!Double.isFinite(tof) || tof <= 0.0) break;

      double tofEff = effectiveTof(tof);

      Twist2d twist = new Twist2d(
          vx * tofEff,
          vy * tofEff,
          omega * tof);

      futurePose = pose.exp(twist); 

      distance = targetPos.minus(futurePose.getTranslation()).getNorm();
    }

    double finalTof = timeForDistance.applyAsDouble(distance);
    Logger.recordOutput("Shooter/Drag/EffectiveTof_" + target.name(), effectiveTof(finalTof));
    Logger.recordOutput("Shooter/Drag/RawTof_" + target.name(), finalTof);
    Logger.recordOutput("Shooter/Drag/DragConstantK", dragConstantK);
    Logger.recordOutput("Shooter/Drag/RobotSpeed",
        Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));

    Translation2d result = targetPos.minus(futurePose.getTranslation());

    if (!Double.isFinite(result.getX())
        || !Double.isFinite(result.getY())
        || result.getNorm() < MIN_NORM) {
      return new Translation2d(MIN_NORM, 0.0);
    }

    return result;
  }

  @AutoLogOutput(key = "Shooter/Planner/RecommendedState")
  public Shooter.State recommendedShooterState() {
    Pose2d current = shooterPoseSupplier.get();
    ChassisSpeeds speeds = shooterVelocitySupplier.get();

    if (current == null || speeds == null) {
      return Shooter.State.STOW_HOOD;
    }

    Translation2d toHub = ShootingTarget.HUB.getLocation().wpi().minus(current.getTranslation());

    double tof = timeForDistance.applyAsDouble(toHub.getNorm());
    double tofEff = effectiveTof(tof);

    double vx = Double.isFinite(speeds.vxMetersPerSecond) ? speeds.vxMetersPerSecond : 0.0;
    double vy = Double.isFinite(speeds.vyMetersPerSecond) ? speeds.vyMetersPerSecond : 0.0;
    double omega =
        Double.isFinite(speeds.omegaRadiansPerSecond) ? speeds.omegaRadiansPerSecond : 0.0;

    Twist2d twist = new Twist2d(vx * tofEff, vy * tofEff, omega * tof);
    Pose2d futureHub = current.exp(twist);

    Logger.recordOutput("Shooter/Planner/FuturePose", futureHub);

    com.team254.lib.geometry.Translation2d pos =
        new com.team254.lib.geometry.Translation2d(current.getTranslation());
    com.team254.lib.geometry.Translation2d futureHubPos =
        new com.team254.lib.geometry.Translation2d(futureHub.getTranslation());

    if (Util.isRed().orElse(false)) {
      pos = pos.mirrorAboutX(FieldConstants.LinesVertical.center);
      futureHubPos = futureHubPos.mirrorAboutX(FieldConstants.LinesVertical.center);
    }

    if (pos.inBounds(config.dangerBounds)) return Shooter.State.STOW_HOOD;
    if (futureHubPos.inBounds(config.dangerBounds)) return Shooter.State.STOW_HOOD;
    if (pos.inBounds(config.dangerBoundsFlipped)) return Shooter.State.STOW_HOOD;
    if (futureHubPos.inBounds(config.dangerBoundsFlipped)) return Shooter.State.STOW_HOOD;
    if (pos.inBounds(config.dangerBoundsFlippedOpponent)) return Shooter.State.STOW_HOOD;
    if (futureHubPos.inBounds(config.dangerBoundsFlippedOpponent)) return Shooter.State.STOW_HOOD;
    if (pos.inBounds(config.dangerBoundsOpponent)) return Shooter.State.STOW_HOOD;
    if (futureHubPos.inBounds(config.dangerBoundsOpponent)) return Shooter.State.STOW_HOOD;

    if (pos.x() < config.hubBounds.maxX()) return Shooter.State.HUB;

    return Shooter.State.LOB;
  }

  /* -------------------------------------------------------------------------- */
  /*                            Public API                                      */
  /* -------------------------------------------------------------------------- */

  /**
   * Sets the linear drag constant k (seconds).
   
   *   - Start with k = Double.POSITIVE_INFINITY (disabled, Galilean behavior)
   *   - If shots overcorrect: inc k
   *   - If shots undercorrect: dec k
   *   - Typical range: 0.3 – 2 seconds
   * @param k Drag constant in seconds. Use Double.POSITIVE_INFINITY to disable.
   */

  public void setDragConstantK(double k) {
    this.dragConstantK = k;
  }

  public double getDragConstantK() {
    return dragConstantK;
  }

  boolean override = false;

  public void setOverride(boolean newOverride) {
    this.override = newOverride;
  }

  public Boolean shouldShoot() {
    if (!atStateSupplier.getAsBoolean()) return false;
    return override;
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

  public void setTimeSinceVisionSupplier(DoubleSupplier timeSinceVision) {
    this.timeSinceVision = timeSinceVision;
  }

  /* -------------------------------------------------------------------------- */
  /*                            Builder                                         */
  /* -------------------------------------------------------------------------- */

  public static Builder builder() {
    return new Builder();
  }

  public static class Builder {
    private Supplier<com.team254.lib.geometry.Pose2d> shooterPoseSupplier;
    private Supplier<com.team254.lib.swerve.ChassisSpeeds> shooterVelocitySupplier;
    private BooleanSupplier atStateSupplier;
    private DoubleSupplier timeSinceVision;

    public Builder shooterPoseSupplier(
        Supplier<com.team254.lib.geometry.Pose2d> shooterPoseSupplier) {
      this.shooterPoseSupplier = shooterPoseSupplier;
      return this;
    }

    public Builder shooterVelocitySupplier(
        Supplier<com.team254.lib.swerve.ChassisSpeeds> shooterVelocitySupplier) {
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
      if (shooterPoseSupplier == null)
        throw new IllegalStateException("shooterPoseSupplier must be set");
      if (shooterVelocitySupplier == null)
        throw new IllegalStateException("shooterVelocitySupplier must be set");
      if (atStateSupplier == null)
        throw new IllegalStateException("atStateSupplier must be set");
      return new ShootingPlanner(this);
    }
  }
}