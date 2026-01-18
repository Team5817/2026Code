package com.team5817.frc2026.planners;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
    
    public ShootingPlanner(Supplier<Pose2d> shooterPoseSupplier){
        InterpolatingDoubleTreeMap hoodLobMap = new InterpolatingDoubleTreeMap();
            hoodLobMap.put(1.0, 10.0);
            hoodLobMap.put(2.0, 12.5);
            hoodLobMap.put(3.5, 15.0);
            hoodLobMap.put(5.0, 18.0);
        InterpolatingDoubleTreeMap turretLobMap = new InterpolatingDoubleTreeMap();
            turretLobMap.put(1.0, 10.0);
            turretLobMap.put(2.0, 12.5);
            turretLobMap.put(3.5, 15.0);
            turretLobMap.put(5.0, 18.0);
        InterpolatingDoubleTreeMap flywheelLobMap = new InterpolatingDoubleTreeMap();
            flywheelLobMap.put(1.0, 10.0);
            flywheelLobMap.put(2.0, 12.5);
            flywheelLobMap.put(3.5, 15.0);
            flywheelLobMap.put(5.0, 18.0);

        InterpolatingDoubleTreeMap hoodHubMap = new InterpolatingDoubleTreeMap();
            hoodHubMap.put(1.0, 10.0);
            hoodHubMap.put(2.0, 12.5);
            hoodHubMap.put(3.5, 15.0);
            hoodHubMap.put(5.0, 18.0);
        InterpolatingDoubleTreeMap turretHubMap = new InterpolatingDoubleTreeMap();
            turretHubMap.put(1.0, 10.0);
            turretHubMap.put(2.0, 12.5);
            turretHubMap.put(3.5, 15.0);
            turretHubMap.put(5.0, 18.0);
        InterpolatingDoubleTreeMap flywheelHubMap = new InterpolatingDoubleTreeMap();
            flywheelHubMap.put(1.0, 10.0);
            flywheelHubMap.put(2.0, 12.5);
            flywheelHubMap.put(3.5, 15.0);
            flywheelHubMap.put(5.0, 18.0);


        Supplier<Translation2d> shooterToHub = () -> shooterPoseSupplier.get().getTranslation().minus(new Translation2d(16.54, 8.02));//TODO Field Constants
        Supplier<Translation2d> shooterToLob = () -> shooterPoseSupplier.get().getTranslation().minus(new Translation2d(16.54, 8.02));//TODO Field Constants
        lobHoodAngleSupplier = () -> {
            double distance = shooterToLob.get().norm();
            return (double)hoodLobMap.get(distance);
        };
        lobTurretAngleSupplier = () -> {
            double distance = shooterToLob.get().norm();
            return (double)turretLobMap.get(distance);
        };
        lobFlywheelSpeedSupplier = () -> {
            double distance = shooterToLob.get().norm();
            return (double)flywheelLobMap.get(distance);
        };
        hubHoodAngleSupplier = () -> {
            double distance = shooterToHub.get().norm();
            return (double)hoodHubMap.get(distance);
        };
        hubTurretAngleSupplier = () -> {
            double distance = shooterToHub.get().norm();
            return (double)turretHubMap.get(distance);
        };
        hubFlywheelSpeedSupplier = () -> {
            double distance = shooterToHub.get().norm();
            return (double)flywheelHubMap.get(distance);
        };
    }
}
