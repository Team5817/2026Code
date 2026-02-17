package com.team5817.lib.drivers.Vision;

import com.team254.lib.geometry.Rotation2d;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.RobotController;
import java.util.*;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ManualVisionIOLimelight implements VisionIO {

  private final Supplier<Pose3d> cameraPoseSupplier;
  private final Supplier<Rotation2d> rotationSupplier;
  private final boolean incomingIsRobotPose;

  private final DoubleArrayPublisher orientationPublisher;
  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;

  public ManualVisionIOLimelight(
      String name,
      Supplier<Pose3d> cameraPoseSupplier,
      Supplier<Rotation2d> rotationSupplier,
      boolean incomingIsRobotPose) {

    var table = NetworkTableInstance.getDefault().getTable(name);

    this.cameraPoseSupplier = cameraPoseSupplier;
    this.rotationSupplier = rotationSupplier;
    this.incomingIsRobotPose = incomingIsRobotPose;

    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    // ---------------- CONNECTION ----------------
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;
    Logger.recordOutput("Vision/Connected", inputs.connected);

    // ---------------- TARGET ANGLES ----------------
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // ---------------- ROBOT YAW PUBLISHED ----------------
    Rotation2d driveYaw = rotationSupplier.get();
    orientationPublisher.accept(new double[] {driveYaw.getDegrees(), 0, 0, 0, 0, 0});
    NetworkTableInstance.getDefault().flush();

    Logger.recordOutput("Vision/DriveYawDeg", driveYaw.getDegrees());

    // ---------------- CAMERA SUPPLIER ----------------
    Pose3d robotToCamera = cameraPoseSupplier.get();

    Logger.recordOutput("Vision/RTC", robotToCamera);

    Transform3d cameraToRobot = new Transform3d(robotToCamera, new Pose3d());

    Logger.recordOutput("Vision/CTR", cameraToRobot);

    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    // ================= MEGATAG 1 =================
    for (var raw : megatag1Subscriber.readQueue()) {
      if (raw.value.length == 0) continue;

      Pose3d llPose = parsePose(raw.value);

      Logger.recordOutput("Vision/LL", llPose);

      Pose3d computed = incomingIsRobotPose ? llPose : llPose.transformBy(cameraToRobot);

      Logger.recordOutput("Vision/Computed", computed);

      poseObservations.add(
          new PoseObservation(
              raw.timestamp * 1.0e-6 - raw.value[6] * 1.0e-3,
              computed,
              raw.value.length >= 18 ? raw.value[17] : 0.0,
              (int) raw.value[7],
              raw.value[9],
              PoseObservationType.MEGATAG_1));
    }

    inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(i -> i).toArray();
  }

  private static Pose3d parsePose(double[] raw) {
    return new Pose3d(
        raw[0],
        raw[1],
        raw[2],
        new Rotation3d(
            Units.degreesToRadians(raw[3]),
            Units.degreesToRadians(raw[4]),
            Units.degreesToRadians(raw[5])));
  }

  @Override
  public void stop() {}

  @Override
  public void start() {}
}
