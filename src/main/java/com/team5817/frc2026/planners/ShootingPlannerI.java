package com.team5817.frc2026.planners;

import java.util.function.DoubleSupplier;

import com.team5817.frc2026.subsystems.Shooter.Shooter;

public interface ShootingPlannerI {
    DoubleSupplier getLobHoodAngleSupplier();
    DoubleSupplier getLobTurretAngleSupplier();
    DoubleSupplier getLobFlywheelSpeedSupplier();
    DoubleSupplier getHubHoodAngleSupplier();
    DoubleSupplier getHubTurretAngleSupplier();
    DoubleSupplier getHubFlywheelSpeedSupplier();
    Shooter.State  recommendedShooterState();
    Boolean shouldShoot();
}
