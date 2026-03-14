package com.team5817.frc2026.autos.Actions;

import com.team254.lib.swerve.ChassisSpeeds;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Drive.Drive.DriveControlState;
import com.team5817.lib.motion.Trajectory;
import edu.wpi.first.wpilibj.Timer;

/** TrajectoryAction is an action that sets a trajectory for the robot to follow. */
public class TrajectoryAction implements Action {

  private Drive mDrive = null;
  private Trajectory mTrajectory;
  private double extraTimeout;
  private Timer timeSinceStart = new Timer();

  /**
   * Constructs a TrajectoryAction with the specified trajectory.
   *
   * @param path The trajectory to follow.
   */
  public TrajectoryAction(Trajectory path, Drive drive) {
    this(path, false, Double.MAX_VALUE, drive);
  }

  public TrajectoryAction(Trajectory path, boolean resetPos, Drive drive) {
    this(path, resetPos, Double.MAX_VALUE, drive);
  }

  public TrajectoryAction(Trajectory path, double extraTimeout, Drive drive) {
    this(path, false, extraTimeout, drive);
  }

  /**
   * Constructs a TrajectoryAction with the specified trajectory and reset position flag.
   *
   * @param path The trajectory to follow.
   * @param resetPos Whether to reset the robot's position.
   */
  public TrajectoryAction(Trajectory path, boolean resetPos, double extraTimeout, Drive drive) {
    this.mTrajectory = path;
    this.mDrive = drive;
    this.extraTimeout = extraTimeout;
  }

  /** Starts the action by setting the trajectory in the drive subsystem. */
  @Override
  public void start() {
    timeSinceStart.reset();
    timeSinceStart.start();
    mDrive.setTrajectory(
        mTrajectory, extraTimeout + mTrajectory.get().trajectory().getTotalTimeSeconds());
  }

  /**
   * Checks if the trajectory is finished.
   *
   * @return true if the trajectory is finished, false otherwise.
   */
  @Override
  public boolean isFinished() {
    // return false;
    return mDrive.isTrajectoryFinished() && timeSinceStart.get() > 0.5;
  }

  /** Updates the action. This method is called periodically while the action is running. */
  @Override
  public void update() {}

  /** Called once when the action is finished. */
  @Override
  public void done() {
    mDrive.setControlState(DriveControlState.OPEN_LOOP);
    mDrive.runVelocity(new ChassisSpeeds());
    System.out.println("Auto: Segement Complete");
  }
}
