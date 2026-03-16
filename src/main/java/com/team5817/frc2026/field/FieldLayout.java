package com.team5817.frc2026.field;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team5817.frc2026.RobotConstants;
import com.team5817.frc2026.field.AlignmentPoint.AlignmentType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.HashMap;
import java.util.List;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b> <br>
 * <br>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldLayout {
  public static double kFieldLength = 17.55;
  public static double kFieldWidth = 8.05;

  public static final AprilTagFieldLayout kTagMap;

  static {
    kTagMap = FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout();
  }

  private static final AlignmentPoint kLeftClimPrep =
      new AlignmentPoint(
          new Translation2d(+(RobotConstants.kBumberSideLength) / 2 + 1.2, -0.38),
          AlignmentType.CLIMB_PREP);
  private static final AlignmentPoint kRightClimbPrep =
      new AlignmentPoint(
          new Translation2d(+(RobotConstants.kBumberSideLength) / 2 + 1.2, 0.48),
          AlignmentType.CLIMB_PREP);

  private static final AlignmentPoint kLeftClimbEntry =
      new AlignmentPoint(
          new Translation2d(RobotConstants.kBumberSideLength / 2 + 1, -0.38),
          AlignmentType.CLIMB_ENTRY);
  private static final AlignmentPoint kRightClimbEntry =
      new AlignmentPoint(
          new Translation2d(RobotConstants.kBumberSideLength / 2 + 1, 0.48),
          AlignmentType.CLIMB_ENTRY);

  public static class Red {

    public static final HashMap<Integer, AprilTag> kAprilTagMap = new HashMap<>();

    public static final AprilTag kAprilTag15 =
        new AprilTag(
            15, List.of(kLeftClimbEntry, kRightClimbEntry, kLeftClimPrep, kRightClimbPrep));

    static {
      kAprilTagMap.put(15, kAprilTag15);
    }
  }

  public static class Blue {
    public static final HashMap<Integer, AprilTag> kAprilTagMap = new HashMap<>();

    public static final AprilTag kAprilTag31 =
        new AprilTag(
            31, List.of(kLeftClimbEntry, kRightClimbEntry, kLeftClimPrep, kRightClimbPrep));

    static {
      kAprilTagMap.put(31, kAprilTag31);
    }
  }

  /**
   * Handles the alliance flip for a given pose.
   *
   * @param blue_pose The pose in the blue alliance frame.
   * @param is_red_alliance Whether the alliance is red.
   * @return The pose in the correct alliance frame.
   */
  public static Pose2d handleAllianceFlip(Pose2d blue_pose, boolean is_red_alliance) {
    if (is_red_alliance) {
      blue_pose = blue_pose.mirrorAboutX(kFieldLength / 2.0);
    }
    return blue_pose;
  }

  /**
   * Handles the alliance flip for a given translation.
   *
   * @param blue_translation The translation in the blue alliance frame.
   * @param is_red_alliance Whether the alliance is red.
   * @return The translation in the correct alliance frame.
   */
  public static Translation2d handleAllianceFlip(
      Translation2d blue_translation, boolean is_red_alliance) {
    if (is_red_alliance) {
      blue_translation = blue_translation.mirrorAboutX(kFieldLength / 2.0);
    }
    return blue_translation;
  }

  /**
   * Handles the alliance flip for a given rotation.
   *
   * @param blue_rotation The rotation in the blue alliance frame.
   * @param is_red_alliance Whether the alliance is red.
   * @return The rotation in the correct alliance frame.
   */
  public static Rotation2d handleAllianceFlip(Rotation2d blue_rotation, boolean is_red_alliance) {
    if (is_red_alliance) {
      blue_rotation = blue_rotation.mirrorAboutX();
    }
    return blue_rotation;
  }

  /**
   * Calculates the distance from the alliance wall.
   *
   * @param x_coordinate The x-coordinate.
   * @param is_red_alliance Whether the alliance is red.
   * @return The distance from the alliance wall.
   */
  public static double distanceFromAllianceWall(double x_coordinate, boolean is_red_alliance) {
    if (is_red_alliance) {
      return kFieldLength - x_coordinate;
    }
    return x_coordinate;
  }

  /**
   * Gets the reef pose based on the alliance.
   *
   * @return The reef pose in the correct alliance frame.
   */
  public static Translation2d getReefPose() {
    var blue = new Translation2d(4.5, 4);
    if (DriverStation.getAlliance().get().equals(Alliance.Red))
      return blue.mirrorAboutX(kFieldLength / 2);
    else return blue;
  }
}
