package com.team5817.frc2025.planners;

import java.util.function.DoubleSupplier;

public interface ShootingPlannerI {
    DoubleSupplier getLobHoodAngleSupplier();
    DoubleSupplier getLobTurretAngleSupplier();
    DoubleSupplier getLobFlywheelSpeedSupplier();
    DoubleSupplier getHubHoodAngleSupplier();
    DoubleSupplier getHubTurretAngleSupplier();
    DoubleSupplier getHubFlywheelSpeedSupplier();
}
