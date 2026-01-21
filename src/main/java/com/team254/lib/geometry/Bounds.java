package com.team254.lib.geometry;

import edu.wpi.first.math.geometry.Translation2d;

/** Simple Bounds class that stores two 2D points constructed from four doubles. */
public class Bounds {
  private final Translation2d a;
  private final Translation2d b;

  /** Construct bounds from four doubles (x1, y1, x2, y2). */
  public Bounds(double x1, double y1, double x2, double y2) {
    this.a = new Translation2d(x1, y1);
    this.b = new Translation2d(x2, y2);
  }

  /** Construct bounds from two Translation2d instances. */
  public Bounds(Translation2d a, Translation2d b) {
    // Translation2d is immutable, but create new instances for parity with previous behavior
    this.a = new Translation2d(a.getX(), a.getY());
    this.b = new Translation2d(b.getX(), b.getY());
  }

  public Translation2d getA() {
    return new Translation2d(a.getX(), a.getY());
  }

  public Translation2d getB() {
    return new Translation2d(b.getX(), b.getY());
  }

  public double minX() {
    return Math.min(a.getX(), b.getX());
  }

  public double minY() {
    return Math.min(a.getY(), b.getY());
  }

  public double maxX() {
    return Math.max(a.getX(), b.getX());
  }

  public double maxY() {
    return Math.max(a.getY(), b.getY());
  }

  public double width() {
    return Math.abs(b.getX() - a.getX());
  }

  public double height() {
    return Math.abs(b.getY() - a.getY());
  }

  @Override
  public String toString() {
    return "Bounds{" + a + ", " + b + "}";
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (!(o instanceof Bounds)) return false;
    Bounds other = (Bounds) o;
    return a.equals(other.a) && b.equals(other.b);
  }

  @Override
  public int hashCode() {
    return 31 * a.hashCode() + b.hashCode();
  }
}
