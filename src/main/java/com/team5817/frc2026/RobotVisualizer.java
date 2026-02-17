package com.team5817.frc2026;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {

  public static Pose3d[] mechanismPoses = new Pose3d[10];

  static {
    for (int i = 0; i < mechanismPoses.length; i++) {
      mechanismPoses[i] = new Pose3d();
    }
  }

  public static void outputTelemetry() {
    Logger.recordOutput("Mechs", mechanismPoses);
  }

  /* ================= TURRET ================= */
  public static void updateTurretPose(double yawDeg) {
    mechanismPoses[0] =
        new Pose3d(
            -0.032,
            0.1,
            0.373,
            new Rotation3d(0, 0, Units.degreesToRadians(yawDeg+180)));
  }

  /* ================= TURRET HOOD ================= */
 public static void updateTurretHoodAngle(double pitchDeg) {
  double pitchRad = Units.degreesToRadians(pitchDeg);

  mechanismPoses[1] =
      mechanismPoses[0].transformBy(
          new Transform3d(
              -0.08446,
              0.0,
              0.065,
              new Rotation3d(0, -pitchRad, 0)));
}

  /* ================= TURRET FLYWHEEL ================= */
  private static double lastFlywheelTime = Timer.getTimestamp();
  private static Rotation3d flywheelRotation = new Rotation3d();

  public static void updateTurretFlywheel(double velocityRadPerSec) {
    double dt = Timer.getTimestamp() - lastFlywheelTime;

    flywheelRotation =
        flywheelRotation.rotateBy(
            new Rotation3d(0, velocityRadPerSec * dt, 0));

    mechanismPoses[2] =
        mechanismPoses[0].transformBy(
            new Transform3d(
                -0.0888,
                0.0,
                0.065,
                flywheelRotation));

    lastFlywheelTime = Timer.getTimestamp();
  }

  /* ================= SPINDEXER LEFT ================= */
  private static double lastLeftTime = Timer.getTimestamp();
  private static Rotation3d leftRotation = new Rotation3d();

  public static void updateSpindexerLeft(double velocity) {
    double dt = Timer.getTimestamp() - lastLeftTime;

    leftRotation =
        leftRotation.rotateBy(new Rotation3d(0, 0, velocity * dt));

    mechanismPoses[3] =
        new Pose3d(
            0.020,
            -0.128,
            0.09,
            leftRotation);

    lastLeftTime = Timer.getTimestamp();
  }

  /* ================= SPINDEXER RIGHT ================= */
  private static double lastRightTime = Timer.getTimestamp();
  private static Rotation3d rightRotation = new Rotation3d();

  public static void updateSpindexerRight(double velocity) {
    double dt = Timer.getTimestamp() - lastRightTime;

    rightRotation =
        rightRotation.rotateBy(new Rotation3d(0, 0, velocity * dt));

    mechanismPoses[4] =
        new Pose3d(
            0.085,
            0.089,
            0.078,
            rightRotation);

    lastRightTime = Timer.getTimestamp();
  }

  /* ================= FIXED HOOD ================= */
  public static void updateFixedHood(double pitchRad) {
    mechanismPoses[5] =
        new Pose3d(
            -0.051,
            -0.3237,
            0.4633,
            new Rotation3d(0, Units.degreesToRadians(17+pitchRad), 0));
  }

  /* ================= FIXED FLYWHEEL ================= */
  public static void updateFixedFlywheel(double pitchRad) {
    mechanismPoses[6] =
        new Pose3d(
            -0.0462,
            -0.142,
            0.4595,
            new Rotation3d(0, -pitchRad, 0));
  }


  /* ================= INTAKE AND HOPPER ================= */
  public static void updateIntake(double posMeters) {
    mechanismPoses[7] = new Pose3d(-0.2667+posMeters,0, 0, new Rotation3d());
    mechanismPoses[8] = mechanismPoses[7].transformBy(new Transform3d( 0.414, 0.0, 0.1945, new Rotation3d(0,posMeters>.25? 0:Units.degreesToRadians(-66),0)));
  }

  /* ================= CLIMB ================= */
  public static void updateClimb(double angleDeg) {
    mechanismPoses[9] =
        new Pose3d(
          -0.261,
            0.0,
            0.4614,
            new Rotation3d(0, Units.degreesToRadians(-140+angleDeg), 0));
  }
}
