package com.team5817.frc2026.planners;

import java.util.function.DoubleSupplier;

public interface ShootingPlannerI {
    DoubleSupplier getLobHoodAngleSupplier();
    DoubleSupplier getLobTurretAngleSupplier();
    DoubleSupplier getLobFlywheelSpeedSupplier();
    DoubleSupplier getHubHoodAngleSupplier();
    DoubleSupplier getHubTurretAngleSupplier();
    DoubleSupplier getHubFlywheelSpeedSupplier();
}
