package com.team5817.frc2026.planners;

import com.team254.lib.geometry.Translation2d;
import com.team5817.frc2026.field.FieldConstants;
import com.team5817.frc2026.subsystems.Shooter.ShooterConstants;
import com.team5817.lib.Util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Generic enum for shooting targets to allow scalable planner APIs. Stores a default location and
 * per-target lookup tables so adding a target is self-contained.
 */
public enum ShootingTarget {
  // Generic LOB target - delegates at runtime to the nearest lob (left or right).
  // Keep a default location so code that expects a Translation2d doesn't NPE during startup.
  LOB(
      ShooterConstants.HOOD_MAP_LOB,
      ShooterConstants.FLYWHEEL_MAP_LOB,
      new Translation2d(0, 0),
      1.0,
      10.0,
      0.5),
  LOBR(
      ShooterConstants.HOOD_MAP_LOB,
      ShooterConstants.FLYWHEEL_MAP_LOB,
      new Translation2d(0, .6),
      10.0,
      30.0,
      1),
  LOBL(
      ShooterConstants.HOOD_MAP_LOB,
      ShooterConstants.FLYWHEEL_MAP_LOB,
      new Translation2d(0, 7.4),
      10.0,
      30.0,
      1),
  HUB(
      ShooterConstants.HOOD_MAP_HUB,
      ShooterConstants.FLYWHEEL_MAP_HUB,
      new Translation2d(4.6, 4),
      4.0,
      30.0,
      0.5);

  private Translation2d location;
  private final InterpolatingDoubleTreeMap hoodMap;
  private final InterpolatingDoubleTreeMap flywheelMap;
  private final double velocityThresholdMetersPerSecond;
  private final double rotationThresholdDegrees;
  private final double timeSinceVisionThresholdSeconds;

  ShootingTarget(
      InterpolatingDoubleTreeMap hoodMap,
      InterpolatingDoubleTreeMap flywheelMap,
      Translation2d location,
      double velocityThresholdMetersPerSecond,
      double rotationThresholdDegrees,
      double timeSinceVisionThresholdSeconds) {
    this.location = location;
    this.hoodMap = hoodMap;
    this.flywheelMap = flywheelMap;
    this.velocityThresholdMetersPerSecond = velocityThresholdMetersPerSecond;
    this.rotationThresholdDegrees = rotationThresholdDegrees;
    this.timeSinceVisionThresholdSeconds = timeSinceVisionThresholdSeconds;
  }

  public Translation2d getLocation() {
    if(Util.isRed().orElse(false))
      return location.mirrorAboutX(FieldConstants.LinesVertical.center);
    else
      return location;
  }

  /** Return the hood lookup table for this target from ShooterConstants. */
  public InterpolatingDoubleTreeMap getHoodMap() {
    return hoodMap;
  }

  /** Return the flywheel lookup table for this target from ShooterConstants. */
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
