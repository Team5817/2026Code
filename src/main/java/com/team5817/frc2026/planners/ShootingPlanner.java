package com.team5817.frc2026.planners;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import com.team5817.frc2026.ActiveTracker;
import com.team5817.frc2026.subsystems.Shooter.Shooter;

import lombok.Getter;

public class ShootingPlanner implements ShootingPlannerI {
    @Getter
    DoubleSupplier lobHoodAngleSupplier;
    @Getter
    DoubleSupplier lobTurretAngleSupplier;
    @Getter
    DoubleSupplier lobFlywheelSpeedSupplier;
    @Getter
    DoubleSupplier hubHoodAngleSupplier;
    @Getter
    DoubleSupplier hubTurretAngleSupplier;
    @Getter
    DoubleSupplier hubFlywheelSpeedSupplier;

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

        Supplier<Translation2d> shooterToHub = () -> this.shooterPoseSupplier.get().getTranslation().minus(hubLocation);
        Supplier<Translation2d> shooterToLob = () -> this.shooterPoseSupplier.get().getTranslation().minus(lobLocation);
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

        Supplier<Translation2d> futureShooterToHub = () -> this.futureShooterPoseSupplier.get().getTranslation().minus(hubLocation);
        Supplier<Translation2d> futureShooterToLob = () -> this.futureShooterPoseSupplier.get().getTranslation().minus(lobLocation);

        lobHoodAngleSupplier = () -> {
            double distance = futureShooterToLob.get().norm();
            return (double)config.hoodLobMap.get(distance);
        };
        lobFlywheelSpeedSupplier = () -> {
            double distance = futureShooterToLob.get().norm();
            return (double)config.flywheelLobMap.get(distance);
        };
        hubHoodAngleSupplier = () -> {
            double distance = futureShooterToHub.get().norm();
            return (double)config.hoodHubMap.get(distance);
        };
        hubFlywheelSpeedSupplier = () -> {
            double distance = futureShooterToHub.get().norm();
            return (double)config.flywheelHubMap.get(distance);
        };

        lobTurretAngleSupplier = () -> {
            Translation2d toLob = futureShooterToLob.get();
            return toLob.direction().getDegrees();
        };

        hubTurretAngleSupplier = () -> {
            Translation2d toHub = futureShooterToHub.get();
            return toHub.direction().getDegrees();
        };
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
            case AIM:
                if(shooterPosVelocitySupplier.get().getTranslation().norm() > config.velocityThresholdMetersPerSecond)//TODO tune velocity threshold
                    return false;

                if(shooterPosVelocitySupplier.get().getRotation().getDegrees() > config.rotationThresholdDegrees)//TODO tune velocity threshold
                    return false;
                break;
        
            case LOB:
                if(shooterPosVelocitySupplier.get().getTranslation().norm() > config.velocityThresholdMetersPerSecond)//TODO tune velocity threshold
                    return false;

                if(shooterPosVelocitySupplier.get().getRotation().getDegrees() > config.rotationThresholdDegrees)//TODO tune velocity threshold
                    return false;
                break;
            case STOW:
                return false;
            default:
                //If we get here, something is wrong
                System.out.println("ShootingPlanner.shouldShoot(): Unknown Shooter State");
                return null;
        }

        if(timeSinceVision.getAsDouble()>config.timeSinceVisionThresholdSeconds)//TODO tune time since vision threshold
            return false;
        return true;
        }
    }