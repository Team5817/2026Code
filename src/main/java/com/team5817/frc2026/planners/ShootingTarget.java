package com.team5817.frc2026.planners;

import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import com.team5817.frc2026.subsystems.Shooter.ShooterConstants;

/**
 * Generic enum for shooting targets to allow scalable planner APIs.
 * Stores a default location and per-target lookup tables so adding a target is self-contained.
 */
public enum ShootingTarget {
    LOBR(ShooterConstants.HOOD_MAP_LOB, ShooterConstants.FLYWHEEL_MAP_LOB, new Translation2d(2.5, 13.5), 1.0, 10.0, 0.5),
    LOBL(ShooterConstants.HOOD_MAP_LOB, ShooterConstants.FLYWHEEL_MAP_LOB, new Translation2d(2.5, 13.5), 1.0, 10.0, 0.5),
    HUB(ShooterConstants.HOOD_MAP_HUB, ShooterConstants.FLYWHEEL_MAP_HUB, new Translation2d(17.5, 13.5), 1.0, 10.0, 0.5);

    private final Translation2d location;
    private final InterpolatingDoubleTreeMap hoodMap;
    private final InterpolatingDoubleTreeMap flywheelMap;
    private final double velocityThresholdMetersPerSecond;
    private final double rotationThresholdDegrees;
    private final double timeSinceVisionThresholdSeconds;

    ShootingTarget(InterpolatingDoubleTreeMap hoodMap, InterpolatingDoubleTreeMap flywheelMap, Translation2d location,
            double velocityThresholdMetersPerSecond, double rotationThresholdDegrees, double timeSinceVisionThresholdSeconds) {
        this.location = location;
        this.hoodMap = hoodMap;
        this.flywheelMap = flywheelMap;
        this.velocityThresholdMetersPerSecond = velocityThresholdMetersPerSecond;
        this.rotationThresholdDegrees = rotationThresholdDegrees;
        this.timeSinceVisionThresholdSeconds = timeSinceVisionThresholdSeconds;
    }

    public Translation2d getLocation() {
        return location;
    }

    /**
     * Return the hood lookup table for this target from ShooterConstants.
     */
    public InterpolatingDoubleTreeMap getHoodMap() {
        return hoodMap;
    }

    /**
     * Return the flywheel lookup table for this target from ShooterConstants.
     */
    public InterpolatingDoubleTreeMap getFlywheelMap() {
        return flywheelMap;
    }

    public double getVelocityThreshold() {
        return velocityThresholdMetersPerSecond;
    }

    public double getRotationThreshold() {
        return rotationThresholdDegrees;
    }

    public double getTimeSinceVisionThreshold() {
        return timeSinceVisionThresholdSeconds;
    }
}
