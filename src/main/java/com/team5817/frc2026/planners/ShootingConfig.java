package com.team5817.frc2026.planners;

import com.team254.lib.geometry.Bounds;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Container for shooting-related maps and tunable constants.
 * Create new configurations for different robots/field conditions or load from disk later.
 */
public class ShootingConfig {
    // Per-target maps were moved to ShootingTarget to make adding targets as simple as
    // adding an enum entry. ShootingConfig keeps shared maps (timeMap) and per-target thresholds.
    public final InterpolatingDoubleTreeMap timeMap;

    public final Bounds dangerBounds;
    public final Bounds hubBounds;

    public ShootingConfig(
            InterpolatingDoubleTreeMap timeMap,
            Bounds dangerBounds,
            Bounds hubBounds) {
        this.timeMap = timeMap;
        this.dangerBounds = dangerBounds;
        this.hubBounds = hubBounds;
    }

    /**
     * Default configuration using the same hardcoded values that were previously in ShootingPlanner.
     */
    public static ShootingConfig defaultConfig() {
        // Per-target maps now live on ShootingTarget. No default per-target maps are
        // created here to avoid duplication; ShootingTarget enum supplies defaults.
        InterpolatingDoubleTreeMap timeMap = new InterpolatingDoubleTreeMap();
            timeMap.put(1.0, 10.0);
            timeMap.put(5.0, 18.0);

        Bounds dangerBounds = new Bounds(0.0, 0.0, 5.0, 27.0);
        Bounds hubBounds = new Bounds(10.0, 0.0, 25.0, 27.0);

        return new ShootingConfig(
            timeMap,
            dangerBounds,
            hubBounds
        );
    }
}
