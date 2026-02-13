package com.team5817.frc2026.planners;

import com.team254.lib.geometry.Bounds;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Container for shooting-related maps and tunable constants. Create new configurations for
 * different robots/field conditions or load from disk later.
 */
public class ShootingConfig {
  // Per-target maps were moved to ShootingTarget to make adding targets as simple as
  // adding an enum entry. ShootingConfig keeps shared maps (timeMap) and per-target thresholds.
  public final InterpolatingDoubleTreeMap timeMap;

  public final Bounds dangerBounds;
  public final Bounds hubBounds;
  public final Bounds blockedBounds;

  public ShootingConfig(InterpolatingDoubleTreeMap timeMap, Bounds dangerBounds, Bounds hubBounds,Bounds blockedBounds) {
    this.timeMap = timeMap;
    this.dangerBounds = dangerBounds;
    this.hubBounds = hubBounds;
    this.blockedBounds = blockedBounds;
  }

  public static ShootingConfig defaultConfig() {
    InterpolatingDoubleTreeMap timeMap = new InterpolatingDoubleTreeMap();
    timeMap.put(1.0, .2);
    Bounds hubBounds = new Bounds(0.0, 0.0, 5.0, 9.0);
    Bounds dangerBounds = new Bounds(4.0, 0.0, 5.3, 1.3);
    Bounds blockedBounds = new Bounds(5,3.5,6, 4.6);
    return new ShootingConfig(timeMap, dangerBounds, hubBounds,blockedBounds);
  }
}
