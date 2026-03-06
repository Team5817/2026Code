package com.team5817.frc2026.autos.Modes;

import com.team5817.frc2026.autos.Actions.ClimbAction;
import com.team5817.frc2026.autos.Actions.ParallelAction;
import com.team5817.frc2026.autos.Actions.ShootAction;
import com.team5817.frc2026.autos.Actions.TrajectoryAction;
import com.team5817.frc2026.autos.AutoBase;
import com.team5817.frc2026.autos.AutoModeFactory.ClimbSelection;
import com.team5817.frc2026.autos.TrajectoryLibrary.l;
import com.team5817.frc2026.planners.ShootingPlanner;
import com.team5817.frc2026.subsystems.Climb.Climb;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;
import java.util.List;

public class NS extends AutoBase {
  // Testing Priority, WIP
  private Drive d;
  private Superstructure su;
  private TrajectorySet t;
  private Climb c;
  private Shooter sh;
  private ClimbSelection climbSelection;
  private ShootingPlanner p;

  public NS(Superstructure s, ClimbSelection climbSelection) {
    this.d = s.mDrive;
    this.su = s;
    this.sh = s.mShooter;
    this.p = sh.getPlanner();
    this.c = s.mClimb;
    this.climbSelection = climbSelection;

    Trajectory SHToNS;
    Trajectory NSToN1;
    Trajectory N1ToC0;
    Trajectory C0ToC1;

    SHToNS = l.trajectories.get("SHToNS");
    NSToN1 = l.trajectories.get("NSToN1");
    N1ToC0 = l.trajectories.get("N1ToC0");
    C0ToC1 = l.trajectories.get("C0ToC1");

    t = new TrajectorySet(false, SHToNS, NSToN1, N1ToC0, C0ToC1);
  }

  @Override
  public void routine() {
    d.simResetWorldPose(t.initalPose());
    sh.followPlan(false);
    sh.setDesiredState(Shooter.State.STOW);

    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(new TrajectoryAction(t.next(), d));

    r(new TrajectoryAction(t.next(), d));
    su.mIntake.stateRequest(Intake.State.IDLE).act();

    r(new ShootAction(6.0, su));

    if (climbSelection == ClimbSelection.SHOULD_CLIMB) {
      r(new ParallelAction(List.of(new ClimbAction(c))));

    } else {
      r(new ShootAction(5, su));
    }

    if (climbSelection == ClimbSelection.SHOULD_CLIMB) {
      r(new TrajectoryAction(t.next(), d));
      r(new ClimbAction(c));
    }
  }
}
