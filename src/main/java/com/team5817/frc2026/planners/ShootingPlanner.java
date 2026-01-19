package com.team5817.frc2026.planners;

import java.util.EnumMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team5817.frc2026.ActiveTracker;
import com.team5817.frc2026.subsystems.Shooter.Shooter;

/**
 * Shooting planner that provides target-keyed suppliers for hood, turret and flywheel.
 */
public class ShootingPlanner implements ShootingPlannerI {

    private final Map<ShootingTarget, DoubleSupplier> hoodAngleSuppliers = new EnumMap<>(ShootingTarget.class);
    private final Map<ShootingTarget, DoubleSupplier> turretAngleSuppliers = new EnumMap<>(ShootingTarget.class);
    private final Map<ShootingTarget, DoubleSupplier> flywheelSpeedSuppliers = new EnumMap<>(ShootingTarget.class);

    private Supplier<Pose2d> futureShooterPoseSupplier;
    private Supplier<Pose2d> shooterPoseSupplier;
    private Supplier<Pose2d> shooterPosVelocitySupplier;
    private BooleanSupplier atStateSupplier;
    private DoubleSupplier timeSinceVision;
    private final ShootingConfig config;

    public ShootingPlanner(Supplier<Pose2d> shooterPoseSupplier, Supplier<Pose2d> shooterVelocitySupplier, BooleanSupplier atStateSupplier, DoubleSupplier timeSinceVision) {
        this(shooterPoseSupplier, shooterVelocitySupplier, atStateSupplier, timeSinceVision, ShootingConfig.defaultConfig());
    }

    public ShootingPlanner(Supplier<Pose2d> shooterPoseSupplier, Supplier<Pose2d> shooterVelocitySupplier, BooleanSupplier atStateSupplier, DoubleSupplier timeSinceVision, ShootingConfig config) {
        this.shooterPosVelocitySupplier = shooterVelocitySupplier;
        this.shooterPoseSupplier = shooterPoseSupplier;
        this.atStateSupplier = atStateSupplier;
        this.timeSinceVision = timeSinceVision;
        this.config = config;

        Supplier<ShootingTarget> closestLobTarget = () -> {
            Translation2d shooterToLobR = this.shooterPoseSupplier.get().getTranslation().minus(ShootingTarget.LOBR.getLocation());
            Translation2d shooterToLobL = this.shooterPoseSupplier.get().getTranslation().minus(ShootingTarget.LOBL.getLocation());
            if (shooterToLobR.norm() < shooterToLobL.norm()) {
                return ShootingTarget.LOBR;
            } else {
                return ShootingTarget.LOBL;
            }
        };
        Supplier<Translation2d> shooterToHub = () -> this.shooterPoseSupplier.get().getTranslation().minus(ShootingTarget.HUB.getLocation());
        Supplier<Translation2d> shooterToLob = () -> this.shooterPoseSupplier.get().getTranslation().minus(closestLobTarget.get().getLocation());
        Supplier<Double> timeSupplier = () -> {
            switch (recommendedShooterState()) {
                case AIM:
                    return (double)config.timeMap.get(shooterToHub.get().norm());
                case LOB:
                    return (double)config.timeMap.get(shooterToLob.get().norm());
                default:
                    return 0.0;
            }
        };

        this.futureShooterPoseSupplier = () -> shooterPoseSupplier.get().transformBy(
            new Pose2d(
                shooterVelocitySupplier.get().getTranslation().times(timeSupplier.get()),
                shooterVelocitySupplier.get().getRotation()
            )
        );

        Supplier<Translation2d> futureShooterToHub = () -> this.futureShooterPoseSupplier.get().getTranslation().minus(ShootingTarget.HUB.getLocation());
        Supplier<Translation2d> futureShooterToLob = () -> this.futureShooterPoseSupplier.get().getTranslation().minus(ShootingTarget.LOB.getLocation());

        Map<ShootingTarget, Supplier<Translation2d>> futureToTarget = new EnumMap<>(ShootingTarget.class);
        futureToTarget.put(ShootingTarget.HUB, futureShooterToHub);
        futureToTarget.put(ShootingTarget.LOB, futureShooterToLob);

        for (ShootingTarget t : ShootingTarget.values()) {
            Supplier<Translation2d> futureTo = futureToTarget.get(t);

            hoodAngleSuppliers.put(t, () -> {
                double distance = futureTo.get().norm();
                return (double) t.getHoodMap().get(distance);
            });

            flywheelSpeedSuppliers.put(t, () -> {
                double distance = futureTo.get().norm();
                return (double) t.getFlywheelMap().get(distance);
            });

            turretAngleSuppliers.put(t, () -> {
                Translation2d to = futureTo.get();
                return to.direction().getDegrees();
            });
        }
    }

    public ShootingPlanner(Supplier<Pose2d> shooterPoseSupplier) {
        this(shooterPoseSupplier, () -> new Pose2d(), () -> true, () -> 0.0);
    }

    public Shooter.State recommendedShooterState(){
        if(futureShooterPoseSupplier.get().getTranslation().inBounds(config.dangerBounds))//inbounds for danger zone
            return Shooter.State.STOW;
        if(futureShooterPoseSupplier.get().getTranslation().inBounds(config.hubBounds) && ActiveTracker.isActive())//inbounds for hub shot
            return Shooter.State.AIM;
        return Shooter.State.LOB;
    }

    @Override
    public Boolean shouldShoot() {
        if(!atStateSupplier.getAsBoolean())
            return false;
        switch (recommendedShooterState()) {
            case AIM: {
                ShootingTarget target = ShootingTarget.HUB;
                if (shooterPosVelocitySupplier.get().getTranslation().norm() > target.getVelocityThreshold()) //TODO tune velocity threshold
                    return false;

                if (shooterPosVelocitySupplier.get().getRotation().getDegrees() > target.getRotationThreshold()) //TODO tune rotation threshold
                    return false;
                break;
            }

            case LOB: {
                ShootingTarget target = ShootingTarget.LOB;
                if (shooterPosVelocitySupplier.get().getTranslation().norm() > target.getVelocityThreshold()) //TODO tune velocity threshold
                    return false;

                if (shooterPosVelocitySupplier.get().getRotation().getDegrees() > target.getRotationThreshold()) //TODO tune rotation threshold
                    return false;
                break;
            }
            case STOW:
                return false;
            default:
                //If we get here, something is wrong
                System.out.println("ShootingPlanner.shouldShoot(): Unknown Shooter State");
                return null;
        }

        // Use the threshold for the current recommended target
        ShootingTarget currentTarget = recommendedShooterState() == Shooter.State.AIM ? ShootingTarget.HUB : ShootingTarget.LOB;
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
}