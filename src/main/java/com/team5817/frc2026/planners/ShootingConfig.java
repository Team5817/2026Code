package com.team5817.frc2026.planners;

import com.team254.lib.geometry.Bounds;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import org.littletonrobotics.junction.Logger;

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
  public final Bounds blockedBounds;
  public final Bounds dangerBoundsOpponent;
  public final Bounds dangerBoundsFlipped;
  public final Bounds dangerBoundsFlippedOpponent;

  public ShootingConfig(
      InterpolatingDoubleTreeMap timeMap,
      Bounds hubBounds,
      Bounds dangerBounds,
      Bounds blockedBounds,
      Bounds dangerBoundsFlipped,
      Bounds dangerBoundsOpponent,
      Bounds dangerBoundsFlippedOpponent) {
    this.timeMap = timeMap;
    this.hubBounds = hubBounds;
    this.dangerBounds = dangerBounds;
    this.blockedBounds = blockedBounds;
    this.dangerBoundsOpponent = dangerBoundsOpponent;
    this.dangerBoundsFlipped = dangerBoundsFlipped;
    this.dangerBoundsFlippedOpponent = dangerBoundsFlippedOpponent;
  }

  public static ShootingConfig defaultConfig() {
    InterpolatingDoubleTreeMap timeMap = new InterpolatingDoubleTreeMap();

    timeMap.put(1.0, 0.16);
    timeMap.put(3.0, 0.24);
    timeMap.put(10.0, 1.0);
    timeMap.put(5.0, 0.34); // TODO add more values

    Bounds hubBounds = new Bounds(0.0, 0.0, 4.6, 8);
    Bounds dangerBounds = new Bounds(3.65, 0, 5.5, 1.3);
    Bounds blockedBounds = new Bounds(5, 3.5, 6, 4.6);
    Bounds dangerBoundsOpponent = dangerBounds.flippedAboutY();
    Bounds dangerBoundsFlipped = dangerBounds.flippedAboutX();
    Bounds dangerBoundsFlippedOpponent = dangerBounds.flippedAboutX().flippedAboutY();

    Logger.recordOutput(
        "Shooting/DangerBounds",
        new double[] {
          dangerBounds.minX(), dangerBounds.minY(), dangerBounds.maxX(), dangerBounds.maxY()
        });
    Logger.recordOutput(
        "Shooting/DangerBoundsOpponent",
        new double[] {
          dangerBoundsOpponent.minX(),
          dangerBoundsOpponent.minY(),
          dangerBoundsOpponent.maxX(),
          dangerBoundsOpponent.maxY()
        });
    Logger.recordOutput(
        "Shooting/DangerBoundsFlipped",
        new double[] {
          dangerBoundsFlipped.minX(),
          dangerBoundsFlipped.minY(),
          dangerBoundsFlipped.maxX(),
          dangerBoundsFlipped.maxY()
        });
    Logger.recordOutput(
        "Shooting/DangerBoundsFlippedOpponent",
        new double[] {
          dangerBoundsFlippedOpponent.minX(),
          dangerBoundsFlippedOpponent.minY(),
          dangerBoundsFlippedOpponent.maxX(),
          dangerBoundsFlippedOpponent.maxY()
        });
    Logger.recordOutput(
        "Shooting/BlockedBounds",
        new double[] {
          blockedBounds.minX(), blockedBounds.minY(), blockedBounds.maxX(), blockedBounds.maxY()
        });
    return new ShootingConfig(
        timeMap,
        hubBounds,
        dangerBounds,
        blockedBounds,
        dangerBoundsOpponent,
        dangerBoundsFlipped,
        dangerBoundsFlippedOpponent);
  }
}
