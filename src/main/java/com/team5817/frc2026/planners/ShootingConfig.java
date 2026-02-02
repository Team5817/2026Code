package com.team5817.frc2026.planners;

import com.team254.lib.geometry.Bounds;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Container for shooting-related maps and tunable constants.
 *
 * This class intentionally contains ONLY shared physics + safety constraints.
 * Per-target tuning (hood, flywheel, thresholds) lives in ShootingTarget.
 */
public class ShootingConfig {

  /* -------------------------------------------------------------------------- */
  /*                               Ballistics                                   */
  /* -------------------------------------------------------------------------- */

  /** Distance (m) → time-of-flight (s) */
  public final InterpolatingDoubleTreeMap timeMap;

  /** Robot → turret transform (meters, radians) */
  public final Transform2d robotToTurret;

  /** Valid shooting distance window (meters) */
  public final double minDistance;
  public final double maxDistance;

  /* -------------------------------------------------------------------------- */
  /*                              Safety / Logic                                 */
  /* -------------------------------------------------------------------------- */

  /** Areas where shooting is forbidden */
  public final Bounds dangerBounds;

  /** Area where hub shots are preferred */
  public final Bounds hubBounds;

  /** Max linear velocity allowed to fire (m/s) */
  public final double maxShootVelocity;

  /** Max angular velocity allowed to fire (deg/s) */
  public final double maxShootOmegaDeg;

  /** Max allowed vision age when firing (s) */
  public final double maxVisionAge;

  /* -------------------------------------------------------------------------- */

  public ShootingConfig(
      InterpolatingDoubleTreeMap timeMap,
      Transform2d robotToTurret,
      double minDistance,
      double maxDistance,
      Bounds dangerBounds,
      Bounds hubBounds,
      double maxShootVelocity,
      double maxShootOmegaDeg,
      double maxVisionAge) {

    this.timeMap = timeMap;
    this.robotToTurret = robotToTurret;
    this.minDistance = minDistance;
    this.maxDistance = maxDistance;
    this.dangerBounds = dangerBounds;
    this.hubBounds = hubBounds;
    this.maxShootVelocity = maxShootVelocity;
    this.maxShootOmegaDeg = maxShootOmegaDeg;
    this.maxVisionAge = maxVisionAge;
  }

  /* -------------------------------------------------------------------------- */
  /*                             Default Config                                  */
  /* -------------------------------------------------------------------------- */

  public static ShootingConfig defaultConfig() {
    InterpolatingDoubleTreeMap timeMap = new InterpolatingDoubleTreeMap();

    // Distance (m) → TOF (s)
    timeMap.put(1.4, 0.90);
    timeMap.put(2.0, 1.05);
    timeMap.put(3.0, 1.10);
    timeMap.put(4.5, 1.15);
    timeMap.put(5.7, 1.18);

    // Robot center → turret pivot
    Transform2d robotToTurret =
        new Transform2d(new Translation2d(0.32, 0.00), new edu.wpi.first.math.geometry.Rotation2d());

    Bounds hubBounds = new Bounds(0.0, 0.0, 5.0, 9.0);
    Bounds dangerBounds = new Bounds(4.0, 0.0, 5.3, 1.3);

    return new ShootingConfig(
        timeMap,
        robotToTurret,
        1.3,  // min distance
        5.8,  // max distance
        dangerBounds,
        hubBounds,
        1.5,  // max linear speed (m/s)
        90.0, // max angular speed (deg/s)
        0.25   // max vision age (s)
    );
  }
}
