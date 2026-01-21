package com.team5817.frc2026.planners;

import com.team5817.frc2026.subsystems.Shooter.Shooter;
import java.util.function.DoubleSupplier;

/**
 * Planner interface that provides suppliers for different shooting targets. Switched to a
 * target-keyed API to make it easier to add new targets.
 */
public interface ShootingPlannerI {
  DoubleSupplier getHoodAngleSupplier(ShootingTarget target);

  DoubleSupplier getTurretAngleSupplier(ShootingTarget target);

  DoubleSupplier getFlywheelSpeedSupplier(ShootingTarget target);

  Shooter.State recommendedShooterState();

  Boolean shouldShoot();
}
