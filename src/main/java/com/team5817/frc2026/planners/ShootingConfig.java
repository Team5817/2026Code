package com.team5817.frc2026.planners;

import org.littletonrobotics.junction.Logger;

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

  public final Bounds hubBounds;
  public final Bounds dangerBounds;
  public final Bounds dangerBoundsOpponent;
  public final Bounds dangerBoundsFlipped;
  public final Bounds dangerBoundsFlippedOpponent;

  public ShootingConfig(InterpolatingDoubleTreeMap timeMap, Bounds hubBounds,Bounds dangerBounds, Bounds dangerBoundsFlipped, 
  Bounds dangerBoundsOpponent, Bounds dangerBoundsFlippedOpponent) {
    this.timeMap = timeMap;
    this.hubBounds = hubBounds;
    this.dangerBounds = dangerBounds;
    this.dangerBoundsOpponent = dangerBoundsOpponent;
    this.dangerBoundsFlipped = dangerBoundsFlipped;
    this.dangerBoundsFlippedOpponent = dangerBoundsFlippedOpponent;
  }

  public static ShootingConfig defaultConfig() {
    InterpolatingDoubleTreeMap timeMap = new InterpolatingDoubleTreeMap();
    timeMap.put(1.0, .2);
    Bounds hubBounds = new Bounds(0.0, 0.0, 4.6, 8);
    Bounds dangerBounds = new Bounds(4.6, 6.73, 5.8,8);
    Bounds dangerBoundsOpponent = new Bounds(4.6, 6.73, 5.8,8).flippedAboutY();
    Bounds dangerBoundsFlipped = new Bounds(4.6, 6.73, 5.8, 8).flippedAboutX();
    Bounds dangerBoundsFlippedOpponent = new Bounds(4.6, 6.73, 5.8, 8).flippedAboutX().flippedAboutY();
    Logger.recordOutput("Shooting/DangerBounds", 
        new double[] { dangerBounds.minX(), dangerBounds.minY(), dangerBounds.maxX(), dangerBounds.maxY() });
    Logger.recordOutput("Shooting/DangerBoundsOpponent", 
        new double[] { dangerBoundsOpponent.minX(), dangerBoundsOpponent.minY(), dangerBoundsOpponent.maxX(), dangerBoundsOpponent.maxY() });
        Logger.recordOutput("Shooting/DangerBoundsFlipped", 
        new double[] { dangerBoundsFlipped.minX(), dangerBoundsFlipped.minY(), dangerBoundsFlipped.maxX(), dangerBoundsFlipped.maxY() });
    Logger.recordOutput("Shooting/DangerBoundsFlippedOpponent", 
        new double[] { dangerBoundsFlippedOpponent.minX(), dangerBoundsFlippedOpponent.minY(), dangerBoundsFlippedOpponent.maxX(), dangerBoundsFlippedOpponent.maxY() });
    return new ShootingConfig(timeMap, hubBounds, dangerBounds, dangerBoundsOpponent, dangerBoundsFlipped, dangerBoundsFlippedOpponent);
  }
}
