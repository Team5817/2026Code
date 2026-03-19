package com.team5817.frc2026.autos.Modes;

import com.team5817.frc2026.autos.Actions.ClimbAction;
import com.team5817.frc2026.autos.Actions.ParallelAction;
import com.team5817.frc2026.autos.Actions.ShootAction;
import com.team5817.frc2026.autos.Actions.TrajectoryAction;
import com.team5817.frc2026.planners.ShootingPlanner;
import com.team5817.frc2026.autos.AutoBase;
import com.team5817.frc2026.autos.TrajectoryLibrary.l;
import com.team5817.frc2026.subsystems.Climb.Climb;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;
import java.util.List;

public class FS extends AutoBase {
  private Drive d;
  private Superstructure su;
  private TrajectorySet t;
  private Climb c;
  private Shooter sh;
  private ShootingPlanner p;

  public FS(Superstructure s, boolean isHumanSide) {
    this.d = s.mDrive;
    this.su = s;
    this.sh = s.mShooter;
    this.c = s.mClimb;
    this.p = sh.getPlanner();

    Trajectory SHToN3 = l.trajectories.get("SHToN3");
    Trajectory N3ToC0 = l.trajectories.get("N3ToC0");
    Trajectory C0ToC1 = l.trajectories.get("C0ToC1");

    t = new TrajectorySet(!isHumanSide, SHToN3, N3ToC0, C0ToC1);
  }

  @Override
  public void routine() {
      d.simResetWorldPose(t.initalPose());
      d.zeroGyro(t.initalPose().getRotation().getDegrees());
      sh.followPlan(false);
      sh.setDesiredState(Shooter.State.STOW_HOOD);
      p.setOverride(false);
      sh.forceStow(true);

      su.mIntake.stateRequest(Intake.State.INTAKING).act();
      r(new TrajectoryAction(t.next(), d));

      su.mIntake.stateRequest(Intake.State.IDLE).act();
      r(new TrajectoryAction(t.next(), d));

      r(new ParallelAction(List.of(
          new ShootAction(5, su, 1),
          new ClimbAction(c)
      )));

      r(new TrajectoryAction(t.next(), d));
      r(new ClimbAction(c));
    }

}
