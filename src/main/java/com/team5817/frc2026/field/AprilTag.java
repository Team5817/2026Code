package com.team5817.frc2026.field;

import com.team254.lib.geometry.Pose2d;
import com.team5817.frc2026.field.AlignmentPoint.AlignmentType;
import java.util.ArrayList;
import java.util.List;

/** Represents an AprilTag on the field. */
public class AprilTag {

  private int id;
  private Pose2d fieldToTag;

  private List<AlignmentType> allTypes = new ArrayList<>();
  private List<AlignmentPoint> allAlignmentPoints;


  public AprilTag(
      int id,
      List<AlignmentPoint> allAlignmentPoints) {
    this.id = id;
    this.fieldToTag = new Pose2d(FieldLayout.kTagMap.getTagPose(id).get().toPose2d());
    this.allAlignmentPoints = allAlignmentPoints;

    for (AlignmentPoint alignments : allAlignmentPoints) {
      for (AlignmentType tagTypes : alignments.getAllowedAllignments()) {
        if (!allTypes.contains(tagTypes)) {
          allTypes.add(tagTypes);
        }
      }
    }
  }

  /**
   * Gets the ID of the AprilTag.
   *
   * @return the ID of the AprilTag
   */
  public int getId() {
    return id;
  }

  /**
   * Gets the pose of the AprilTag relative to the field.
   *
   * @return the pose of the AprilTag relative to the field
   */
  public Pose2d getFieldToTag() {
    return fieldToTag;
  }

  /**
   * Gets all allowable alignment types for the AprilTag.
   *
   * @return a list of allowable alignment types
   */
  public List<AlignmentType> getAllAllowableAllignments() {
    return allTypes;
  }

  /**
   * Gets all alignment points for the AprilTag.
   *
   * @return a list of alignment points
   */
  public List<AlignmentPoint> getAllAlignmentPoints() {
    return allAlignmentPoints;
  }
}
