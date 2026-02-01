package com.team5817.lib.drivers.Vision;

import com.team254.lib.geometry.Rotation2d;
import com.team5817.lib.vision.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

/** IO implementation for a Limelight that publishes camera poses. We manually transform camera -> robot. */
public class ManualVisionIOLimelight implements VisionIO {
  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<Pose3d> cameraPoseSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag1Subscriber;
  private final DoubleArraySubscriber megatag2Subscriber;
  private final String kName;
  private final boolean incomingIsRobotPose;

  /**
   * Creates a new ManualVisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   * @param cameraPoseSupplier Supplier that returns the camera pose expressed in robot coordinates
   *     (robot->camera). This will be inverted to compute camera->robot for applying to camera
   *     poses reported by the Limelight.
   * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
   */
  public ManualVisionIOLimelight(
      String name,
      Supplier<Pose3d> cameraPoseSupplier,
      Supplier<Rotation2d> rotationSupplier,
      boolean incomingIsRobotPose) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    this.rotationSupplier = rotationSupplier;
    this.cameraPoseSupplier = cameraPoseSupplier;
    this.kName = name;
    this.incomingIsRobotPose = incomingIsRobotPose;
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
    megatag2Subscriber =
        table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    // no-op name kept out; kName unused to avoid warnings
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected =
        ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    // Update target observation (2D angles)
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Publish robot orientation for MegaTag 2 support (same behavior as original)
    orientationPublisher.accept(
        new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
    NetworkTableInstance.getDefault().flush(); // Increases network traffic but recommended by Limelight

    // Read new pose observations from NetworkTables. Limelight is reporting CAMERA pose; we
    // transform each camera-published pose into robot pose using the provided camera pose
    // supplier (robot->camera). We invert that to get camera->robot.
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var rawSample : megatag1Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }

      // Parse pose reported by Limelight. If the Limelight is publishing robot poses (common
      // when using botpose_wpiblue), use it directly. If it publishes camera poses instead,
      // transform using the provided robot->camera supplier.
      Pose3d cameraPose = parsePose(rawSample.value);
    Logger.recordOutput("turretLight" + "/RawCameraPose", cameraPose);
    // Log scalar rotations for easier debugging
    Rotation3d rawRot = cameraPose.getRotation();
    Logger.recordOutput("turretLight" + "/RawRollDeg", Math.toDegrees(rawRot.getX()));
    Logger.recordOutput("turretLight" + "/RawPitchDeg", Math.toDegrees(rawRot.getY()));
    Logger.recordOutput("turretLight" + "/RawYawDeg", Math.toDegrees(rawRot.getZ()));
      Pose3d robotPose;
      if (incomingIsRobotPose) {
        robotPose = cameraPose;
      } else {
        Pose3d sup = cameraPoseSupplier.get();
        Transform3d cameraToRobot =
            new Transform3d(sup.getTranslation(), sup.getRotation()).inverse();
    Logger.recordOutput("turretLight" + "/SupplierRobotToCamera", sup);
    Rotation3d supRot = sup.getRotation();
    Logger.recordOutput("turretLight" + "/SupplierRollDeg", Math.toDegrees(supRot.getX()));
    Logger.recordOutput("turretLight" + "/SupplierPitchDeg", Math.toDegrees(supRot.getY()));
    Logger.recordOutput("turretLight" + "/SupplierYawDeg", Math.toDegrees(supRot.getZ()));
        // Log the computed camera->robot transform for debugging
        Logger.recordOutput("turretLight" + "/CameraToRobotTransform", cameraToRobot);
        Translation3d ctrTrans = cameraToRobot.getTranslation();
        Rotation3d ctrRot = cameraToRobot.getRotation();
        Logger.recordOutput("turretLight" + "/CameraToRobotTransX", ctrTrans.getX());
        Logger.recordOutput("turretLight" + "/CameraToRobotTransY", ctrTrans.getY());
        Logger.recordOutput("turretLight" + "/CameraToRobotTransZ", ctrTrans.getZ());
        Logger.recordOutput("turretLight" + "/CameraToRobotRollDeg", Math.toDegrees(ctrRot.getX()));
        Logger.recordOutput("turretLight" + "/CameraToRobotPitchDeg", Math.toDegrees(ctrRot.getY()));
        Logger.recordOutput("turretLight" + "/CameraToRobotYawDeg", Math.toDegrees(ctrRot.getZ()));

        robotPose = cameraPose.transformBy(cameraToRobot);
      }
    Logger.recordOutput("turretLight" + "/ComputedRobotPose", robotPose);
    Rotation3d compRot = robotPose.getRotation();
    Logger.recordOutput("turretLight" + "/ComputedRollDeg", Math.toDegrees(compRot.getX()));
    Logger.recordOutput("turretLight" + "/ComputedPitchDeg", Math.toDegrees(compRot.getY()));
    Logger.recordOutput("turretLight" + "/ComputedYawDeg", Math.toDegrees(compRot.getZ()));

    poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate (robot pose computed from camera)
              robotPose,

              // Ambiguity, using only the first tag because ambiguity isn't applicable for multitag
              rawSample.value.length >= 18 ? rawSample.value[17] : 0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_1));
    }

    for (var rawSample : megatag2Subscriber.readQueue()) {
      if (rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i += 7) {
        tagIds.add((int) rawSample.value[i]);
      }

    Pose3d cameraPose = parsePose(rawSample.value);
    Pose3d robotPose;
    if (incomingIsRobotPose) {
      robotPose = cameraPose;
    } else {
      Transform3d cameraToRobot =
          new Transform3d(cameraPoseSupplier.get().getTranslation(), cameraPoseSupplier.get().getRotation())
              .inverse();
      robotPose = cameraPose.transformBy(cameraToRobot);
    }

      poseObservations.add(
          new PoseObservation(
              // Timestamp, based on server timestamp of publish and latency
              rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

              // 3D pose estimate (robot pose computed from camera)
              robotPose,

              // Ambiguity, zeroed because the pose is already disambiguated
              0.0,

              // Tag count
              (int) rawSample.value[7],

              // Average tag distance
              rawSample.value[9],

              // Observation type
              PoseObservationType.MEGATAG_2));
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  /** Parses the 3D pose from a Limelight botpose array (assumes x,y,z,rotX,rotY,rotZ in degrees). */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }

  @Override
  public void stop() {
  }

  @Override
  public void start() {
  }
}
