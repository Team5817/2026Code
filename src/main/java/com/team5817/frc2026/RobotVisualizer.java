package com.team5817.frc2026;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
  public static Pose3d[] mechanismPoses = new Pose3d[5];

  static {
    for (int i = 0; i < 5; i++) {
      mechanismPoses[i] = new Pose3d();
    }
  }

  public static void outputTelemetry() {
    Logger.recordOutput("Mechs", mechanismPoses);
  }

  /* ================= TURRET ================= */
  public static void updateTurretPose(double position) {
    Pose3d current =
        new Pose3d(
            -0.032,
            0.1,
            0.373,
            new Rotation3d(0, 0, Units.degreesToRadians(position)));

    mechanismPoses[0] = current;
  }

  /* ================= HOOD ================= */
  public static void updateHoodAngle(double position) {
    Pose3d hoodPose =
        mechanismPoses[0].transformBy(
            new Transform3d(
                0.044 - (-0.032),
                0.0,
                0.456 - 0.373,
                new Rotation3d(0, Units.degreesToRadians(position), 0)));

    mechanismPoses[1] = hoodPose;
  }

  /* ================= FLYWHEEL ================= */
  private static double lastFlywheelTime = Timer.getTimestamp();

  public static void updateFlyWheel(double velocity) {
    double dt = Timer.getTimestamp() - lastFlywheelTime;

    Pose3d flywheelPose =
        mechanismPoses[0].transformBy(
            new Transform3d(
                0.044 - (-.0322),
                0.0,
                0.456 - 0.373,
                new Rotation3d(Units.radiansToRotations(0), velocity * dt, 0)));

    mechanismPoses[2] = flywheelPose;
    lastFlywheelTime = Timer.getTimestamp();
  }

  /* ================= SPINDEXER LEFT ================= */
  private static double lastLeftTime = Timer.getTimestamp();

  public static void updateSpindexerLeft(double velocity) {
    double dt = Timer.getTimestamp() - lastLeftTime;

    Pose3d spindexer1Pose =
        new Pose3d(
            0.048,
            -.15,
            .02,
            mechanismPoses[3]
                .getRotation()
                .rotateBy(new Rotation3d(0, 0, dt * velocity)));

    Logger.recordOutput("SpindexerLeft/velocity", velocity);

    mechanismPoses[3] = spindexer1Pose;
    lastLeftTime = Timer.getTimestamp();
  }

  /* ================= SPINDEXER RIGHT ================= */
  private static double lastRightTime = Timer.getTimestamp();

  public static void updateSpindexerRight(double velocity) {
    double dt = Timer.getTimestamp() - lastRightTime;

    Pose3d spindexer2Pose =
        new Pose3d(
            0.135,
            0.15,
            0.013,
            mechanismPoses[4]
                .getRotation()
                .rotateBy(new Rotation3d(0, 0, dt * velocity)));

    mechanismPoses[4] = spindexer2Pose;
    lastRightTime = Timer.getTimestamp();
  }
}
