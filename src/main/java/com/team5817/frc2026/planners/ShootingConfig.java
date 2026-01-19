package com.team5817.frc2026.planners;

import com.team254.lib.geometry.Bounds;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Container for shooting-related maps and tunable constants.
 * Create new configurations for different robots/field conditions or load from disk later.
 */
public class ShootingConfig {
    public final InterpolatingDoubleTreeMap hoodLobMap;
    public final InterpolatingDoubleTreeMap flywheelLobMap;
    public final InterpolatingDoubleTreeMap hoodHubMap;
    public final InterpolatingDoubleTreeMap flywheelHubMap;
    public final InterpolatingDoubleTreeMap timeMap;

    public final Bounds dangerBounds;
    public final Bounds hubBounds;

    public final double velocityThresholdMetersPerSecond;
    public final double rotationThresholdDegrees;
    public final double timeSinceVisionThresholdSeconds;

    public ShootingConfig(
            InterpolatingDoubleTreeMap hoodLobMap,
            InterpolatingDoubleTreeMap flywheelLobMap,
            InterpolatingDoubleTreeMap hoodHubMap,
            InterpolatingDoubleTreeMap flywheelHubMap,
            InterpolatingDoubleTreeMap timeMap,
            Bounds dangerBounds,
            Bounds hubBounds,
            double velocityThresholdMetersPerSecond,
            double rotationThresholdDegrees,
            double timeSinceVisionThresholdSeconds) {
        this.hoodLobMap = hoodLobMap;
        this.flywheelLobMap = flywheelLobMap;
        this.hoodHubMap = hoodHubMap;
        this.flywheelHubMap = flywheelHubMap;
        this.timeMap = timeMap;
        this.dangerBounds = dangerBounds;
        this.hubBounds = hubBounds;
        this.velocityThresholdMetersPerSecond = velocityThresholdMetersPerSecond;
        this.rotationThresholdDegrees = rotationThresholdDegrees;
        this.timeSinceVisionThresholdSeconds = timeSinceVisionThresholdSeconds;
    }

    /**
     * Default configuration using the same hardcoded values that were previously in ShootingPlanner.
     */
    public static ShootingConfig defaultConfig() {
        InterpolatingDoubleTreeMap hoodLobMap = new InterpolatingDoubleTreeMap();
            hoodLobMap.put(1.0, 10.0);
            hoodLobMap.put(2.0, 12.5);
            hoodLobMap.put(3.5, 15.0);
            hoodLobMap.put(5.0, 18.0);
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
        InterpolatingDoubleTreeMap flywheelHubMap = new InterpolatingDoubleTreeMap();
            flywheelHubMap.put(1.0, 10.0);
            flywheelHubMap.put(2.0, 12.5);
            flywheelHubMap.put(3.5, 15.0);
            flywheelHubMap.put(5.0, 18.0);
        InterpolatingDoubleTreeMap timeMap = new InterpolatingDoubleTreeMap();
            timeMap.put(1.0, 10.0);
            timeMap.put(5.0, 18.0);

        Bounds dangerBounds = new Bounds(0.0, 0.0, 5.0, 27.0);
        Bounds hubBounds = new Bounds(10.0, 0.0, 25.0, 27.0);

        double velocityThreshold = 1.0;
        double rotationThreshold = 10.0;
        double timeSinceVisionThreshold = 0.5;

        return new ShootingConfig(
            hoodLobMap,
            flywheelLobMap,
            hoodHubMap,
            flywheelHubMap,
            timeMap,
            dangerBounds,
            hubBounds,
            velocityThreshold,
            rotationThreshold,
            timeSinceVisionThreshold
        );
    }
}
