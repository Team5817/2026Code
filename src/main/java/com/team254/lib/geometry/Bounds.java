package com.team254.lib.geometry;

import com.team5817.frc2026.field.FieldConstants;

/** Simple Bounds class that stores two 2D points constructed from four doubles. */
public class Bounds {
  private final Translation2d bottomLft;
  private final Translation2d topright;

  /** Construct bounds from four doubles (x1, y1, x2, y2). */
  public Bounds(double x1, double y1, double x2, double y2) {
    this.bottomLft = new Translation2d(x1, y1);
    this.topright = new Translation2d(x2, y2);
    this.bottomRight = new Translation2d(x1, y1);
    this.topLft = new Translation2d(x2, y2);
  }

  /** Construct bounds from two Translation2d instances. */
  public Bounds(Translation2d a, Translation2d b) {
    // Translation2d is immutable, but create new instances for parity with previous behavior
    this.bottomLft = new Translation2d(a.x(), a.y());
    this.topright = new Translation2d(b.x(), b.y());
    this.bottomRight = new Translation2d(b.x(), a.y());
    this.topLft = new Translation2d(a.x(), b.y());
  }

  public Bounds flippedAboutX() {
    Translation2d topLft = topright.mirrorAboutX(FieldConstants.fieldLength / 2);
    Translation2d bottomRight = bottomLft.mirrorAboutX(FieldConstants.fieldLength / 2);
    Translation2d topright = new Translation2d(bottomRight.x(), topLft.y());
    Translation2d bottomLft = new Translation2d(topLft.x(), bottomRight.y());

    return new Bounds(bottomLft, topright);
  }

  private final Translation2d bottomRight;
  private final Translation2d topLft;

  public Bounds flippedAboutY() {
    Translation2d topRight = topLft.mirrorAboutY(FieldConstants.fieldWidth / 2);
    Translation2d bottomLft = bottomRight.mirrorAboutY(FieldConstants.fieldWidth / 2);
    Translation2d topLft = new Translation2d(bottomLft.x(), topRight.y());
    Translation2d bottomRight = new Translation2d(topRight.x(), bottomLft.y());

    return new Bounds(topLft, bottomRight);
  }

  public Translation2d getBottomLft() {
    return new Translation2d(bottomLft.x(), bottomLft.y());
  }

  public Translation2d getTopright() {
    return new Translation2d(topright.x(), topright.y());
  }

  public double minX() {
    return Math.min(bottomLft.x(), topright.x());
  }

  public double minY() {
    return Math.min(bottomLft.y(), topright.y());
  }

  public double maxX() {
    return Math.max(bottomLft.x(), topright.x());
  }

  public double maxY() {
    return Math.max(bottomLft.y(), topright.y());
  }

  public double width() {
    return Math.abs(topright.x() - bottomLft.x());
  }

  public double height() {
    return Math.abs(topright.y() - bottomLft.y());
  }

  @Override
  public String toString() {
    return "Bounds{" + bottomLft + ", " + topright + "}";
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (!(o instanceof Bounds)) return false;
    Bounds other = (Bounds) o;
    return bottomLft.equals(other.bottomLft) && topright.equals(other.topright);
  }

  @Override
  public int hashCode() {
    return 31 * bottomLft.hashCode() + topright.hashCode();
  }
}
