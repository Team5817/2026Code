package com.team5817.frc2026.planners;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.DoubleUnaryOperator;
import java.util.function.Supplier;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team5817.frc2026.ActiveTracker;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Shooting planner that provides target-keyed suppliers for hood, turret and flywheel.
 */
public class ShootingPlanner implements ShootingPlannerI {

    private final Map<ShootingTarget, DoubleSupplier> hoodAngleSuppliers = new EnumMap<>(ShootingTarget.class);
    private final Map<ShootingTarget, DoubleSupplier> turretAngleSuppliers = new EnumMap<>(ShootingTarget.class);
    private final Map<ShootingTarget, DoubleSupplier> flywheelSpeedSuppliers = new EnumMap<>(ShootingTarget.class);

    private Supplier<Pose2d> shooterPoseSupplier;
    private Supplier<ChassisSpeeds> shooterPosVelocitySupplier;
    private BooleanSupplier atStateSupplier;
    private DoubleSupplier timeSinceVision;
    private DoubleUnaryOperator timeForDistance;
    private final ShootingConfig config;

    public ShootingPlanner(Supplier<Pose2d> shooterPoseSupplier, Supplier<ChassisSpeeds> shooterVelocitySupplier, BooleanSupplier atStateSupplier, DoubleSupplier timeSinceVision) {
        this.shooterPoseSupplier = shooterPoseSupplier;
        this.shooterPosVelocitySupplier = shooterVelocitySupplier;
        this.atStateSupplier = atStateSupplier;
        this.timeSinceVision = timeSinceVision;
        this.config = ShootingConfig.defaultConfig();

        this.timeForDistance = (distance) -> {
            Double val = (Double) config.timeMap.get(distance);
            return val != null ? val : 0.0;
        };

        Supplier<Translation2d> futureShooterToHub = () -> {
            Pose2d current = this.shooterPoseSupplier.get();
            double distance = ShootingTarget.HUB.getLocation().minus(current.getTranslation()).norm();
            Pose2d future = predictFuturePose(distance);
            return ShootingTarget.HUB.getLocation().minus(future.getTranslation());
        };

        Supplier<Translation2d> futureShooterToLobL = () -> {
            Pose2d current = this.shooterPoseSupplier.get();
            double distance = ShootingTarget.LOBL.getLocation().minus(current.getTranslation()).norm();
            Pose2d future = predictFuturePose(distance);
            return ShootingTarget.LOBL.getLocation().minus(future.getTranslation());
        };

        Supplier<Translation2d> futureShooterToLobR = () -> {
            Pose2d current = this.shooterPoseSupplier.get();
            double distance = ShootingTarget.LOBR.getLocation().minus(current.getTranslation()).norm();
            Pose2d future = predictFuturePose(distance);
            return ShootingTarget.LOBR.getLocation().minus(future.getTranslation());
        };

        Supplier<ShootingTarget> futureClosestLob = () -> futureShooterToLobR.get().norm() < futureShooterToLobL.get().norm() ? ShootingTarget.LOBR : ShootingTarget.LOBL;

        Map<ShootingTarget, Supplier<Translation2d>> futureToTarget = new EnumMap<>(ShootingTarget.class);
        futureToTarget.put(ShootingTarget.HUB, futureShooterToHub);
        futureToTarget.put(ShootingTarget.LOBL, futureShooterToLobL);
        futureToTarget.put(ShootingTarget.LOBR, futureShooterToLobR);
        // For the generic LOB key, point to whichever lob is predicted to be closest using that lob's own horizon.
        futureToTarget.put(ShootingTarget.LOB, () -> {
            ShootingTarget chosen = futureClosestLob.get();
            Pose2d current = this.shooterPoseSupplier.get();
            double distance = chosen.getLocation().minus(current.getTranslation()).norm();
            Pose2d future = predictFuturePose(distance);
            return chosen.getLocation().minus(future.getTranslation());
        });

        for (ShootingTarget t : ShootingTarget.values()) {
            Supplier<Translation2d> futureTo = futureToTarget.get(t);

            hoodAngleSuppliers.put(t, () -> {
                Translation2d to = futureTo.get();

                double distance = to.norm();
                Double val = t.getHoodMap().get(distance);
                return val;
            });

            flywheelSpeedSuppliers.put(t, () -> {
                Translation2d to = futureTo.get();

                double distance = to.norm();
                Double val = t.getFlywheelMap().get(distance);
                return val;
            });

            turretAngleSuppliers.put(t, () -> {
                Translation2d to = futureTo.get();
                return to.direction().getDegrees();
            });
        }
    }
    @AutoLogOutput(key= "Shooter/Recomended State")
    public Shooter.State recommendedShooterState(){
        Pose2d current = this.shooterPoseSupplier.get();
        Translation2d shooterToHub = ShootingTarget.HUB.getLocation().minus(current.getTranslation());
        double hubDist = shooterToHub.norm();
        Pose2d futureHub = predictFuturePose(hubDist);

        Logger.recordOutput("Shooter/FuturePose", futureHub.wpi());

        if (current.getTranslation().inBounds(config.dangerBounds))
            return Shooter.State.STOW;

        if (futureHub.getTranslation().inBounds(config.hubBounds) && ActiveTracker.isActive())
            return Shooter.State.HUB;

        return Shooter.State.LOB;
    }

    @Override
    public Boolean shouldShoot() {
        if(!atStateSupplier.getAsBoolean())
            return false;
        // Compute once to avoid repeated supplier calls and potential side-effects
        Shooter.State recommended = recommendedShooterState();
        // Cache the velocity/rotation supplier result to avoid multiple supplier.get() calls
        Pose2d vel = shooterPosVelocitySupplier.get().toPose2d();
        if (vel == null) {
            // Missing velocity measurement — refuse to shoot and record telemetry for debugging.
            Logger.recordOutput("Shooter/ShouldShoot", "Missing shooter velocity");
            return false;
        }

        switch (recommended) {
            case HUB: {
                ShootingTarget target = ShootingTarget.HUB;
                if (vel.getTranslation().norm() > target.getVelocityThreshold()) //TODO tune velocity threshold
                    return false;
                if (Math.abs(vel.getRotation().getDegrees()) > target.getRotationThreshold()) //TODO tune rotation threshold
                    return false;
                break;
            }

            case LOB: {
                ShootingTarget target = ShootingTarget.LOB;
                if (vel.getTranslation().norm() > target.getVelocityThreshold()) //TODO tune velocity threshold
                    return false;
                if (Math.abs(vel.getRotation().getDegrees()) > target.getRotationThreshold()) //TODO tune rotation threshold
                    return false;
                break;
            }
            case STOW:
                return false;
            default:
                Logger.recordOutput("ShootingPlanner/UnknownState", "Unknown shooter state: " + String.valueOf(recommended));
                return false;
        }

        // Use the threshold for the current recommended target
        ShootingTarget currentTarget = recommended == Shooter.State.HUB ? ShootingTarget.HUB : ShootingTarget.LOB;
        if (timeSinceVision.getAsDouble() > currentTarget.getTimeSinceVisionThreshold()) //TODO tune time since vision threshold
            return false;
        return true;
    }

    @Override
    public DoubleSupplier getHoodAngleSupplier(ShootingTarget target) {
        return hoodAngleSuppliers.get(target);
    }

    @Override
    public DoubleSupplier getTurretAngleSupplier(ShootingTarget target) {
        return turretAngleSuppliers.get(target);
    }

    @Override
    public DoubleSupplier getFlywheelSpeedSupplier(ShootingTarget target) {
        return flywheelSpeedSuppliers.get(target);
    }

    private Pose2d predictFuturePose(double distance) {
        Pose2d current = this.shooterPoseSupplier.get();
        Pose2d vel = this.shooterPosVelocitySupplier.get().toPose2d();
        if (current == null) {
            return new Pose2d();
        }
        if (vel == null) {
            return current;
        }
        double dt = this.timeForDistance.applyAsDouble(distance);
        return current.transformBy(new Pose2d(vel.getTranslation().times(dt), Rotation2d.identity()));
    }
}