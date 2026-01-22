package com.team5817.frc2026;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
  public static Pose3d[] mechanismPoses = new Pose3d[6];

  static {
    for (int i = 0; i < 6; i++) {
      mechanismPoses[i] = new Pose3d();
    }
  }

  public static void outputTelemetry() {
    Logger.recordOutput("Mechs", mechanismPoses);
  }

  public static void updateIntakeAngle(double position) {
    mechanismPoses[0] =
        new Pose3d(
            new Translation3d(-.314, 0, .272),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(position),
                Units.degreesToRadians(0)));
  }

  public static void updateTurretPose(double position) {
    Pose3d current = new Pose3d(-0.032, 0.1, 0.373, new Rotation3d(0, 0, 0));

    mechanismPoses[0] = current;
  }

  public static void updateHoodAngle(double position) {
    Pose3d hoodPose =
        mechanismPoses[0].transformBy(
            new Transform3d(
                0.044 - (-0.032), // relative X
                0.0,
                0.456 - 0.373, // relative Z
                new Rotation3d(0, 0, 0) //  z for pitch
                ));

    mechanismPoses[1] = hoodPose;
  }

  public static void updateFlyWheel(double position) {
    Pose3d flywheelPose =
        mechanismPoses[0].transformBy(
            new Transform3d(0.044 - (-.0322), 0.0, 0.456 - 0.373, new Rotation3d()));
    mechanismPoses[2] = flywheelPose;
  }

  private static double lastLeftTime = 0;

  public static void updateSpindexerLeft(double velocity) { // 1
    double dt = Timer.getTimestamp() - lastLeftTime;
    Pose3d spindexer1Pose =
        new Pose3d(
            0.048,
            -.15,
            -0.013,
            mechanismPoses[3]
                .getRotation()
                .rotateBy(new Rotation3d(0, 0, Units.radiansToRotations(dt * velocity))));
    mechanismPoses[3] = spindexer1Pose;
    lastLeftTime = Timer.getTimestamp();
  }

  private static double lastRightTime = 0;

  public static void updateSpindexerRight(double velocity) { // 2
    double dt = Timer.getTimestamp() - lastRightTime;
    Pose3d spindexer2Pose =
        new Pose3d(
            0.135,
            0.15,
            0.013,
            mechanismPoses[4]
                .getRotation()
                .rotateBy(new Rotation3d(0, 0, Units.radiansToRotations(velocity * dt))));
    mechanismPoses[4] = spindexer2Pose;
    lastRightTime = Timer.getTimestamp();
  }
}
