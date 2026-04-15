package com.team5817.frc2026.autos.Modes;

import com.team254.lib.geometry.Pose2d;
import com.team5817.frc2026.autos.Actions.ShootAction;
import com.team5817.frc2026.autos.Actions.TrajectoryAction;
import com.team5817.frc2026.autos.AutoBase;
import com.team5817.frc2026.autos.TrajectoryLibrary.l;
import com.team5817.frc2026.planners.ShootingPlanner;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;

public class CD extends AutoBase {
  private Drive d;
  private Superstructure su;
  private TrajectorySet t;
  private Shooter sh;
  private ShootingPlanner p;

  public CD(Superstructure s) {
    this.d = s.mDrive;
    this.su = s;
    this.sh = s.mShooter;
    this.p = sh.getPlanner();

    Trajectory COToDP = l.trajectories.get("COToDP");
    Trajectory DPToDE = l.trajectories.get("DPToDE");
    Trajectory DEToDP = l.trajectories.get("DEToDP");
    t = new TrajectorySet(COToDP, DPToDE, DEToDP);
  }

  @Override
  public void routine() {
    d.simResetWorldPose(t.initalPose());
    d.zeroGyro(t.initalPose().getRotation().getDegrees());
    d.setPose(t.initalPose().transformBy(new Pose2d(0, 1, 0)));
    sh.followPlan(false);

    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(new TrajectoryAction(t.next(), 1, d));
    r(new TrajectoryAction(t.next(), 1, d));
    r(new TrajectoryAction(t.next(), 1, d));
    r(new ShootAction(6, su, 2));
  }
}
