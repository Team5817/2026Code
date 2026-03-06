package com.team5817.frc2026.autos.Modes;

import java.util.List;

import com.team5817.frc2026.autos.AutoBase;
import com.team5817.frc2026.autos.AutoModeFactory.ClimbSelection;
import com.team5817.frc2026.autos.TrajectoryLibrary.l;
import com.team5817.frc2026.autos.Actions.ClimbAction;
import com.team5817.frc2026.autos.Actions.ParallelAction;
import com.team5817.frc2026.autos.Actions.ShootAction;
import com.team5817.frc2026.autos.Actions.TrajectoryAction;
import com.team5817.frc2026.planners.ShootingPlanner;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.frc2026.subsystems.Climb.Climb;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;

public class NSwipe extends AutoBase {
  private Drive d;
  private Superstructure su;
  private TrajectorySet t;
  private Climb c;
  private Shooter sh;
  private ClimbSelection climbSelection;
  private ShootingPlanner p;

  public NSwipe(Superstructure s, ClimbSelection climbSelection, boolean isHumanSide) {
    this.d = s.mDrive;
    this.su = s;
    this.sh = s.mShooter;
    this.p = sh.getPlanner();
    this.c = s.mClimb;
    this.climbSelection = climbSelection;

    Trajectory SHToNE;
    Trajectory NEToSH;
    Trajectory SHToNE2;
    Trajectory NE2ToSH;
    Trajectory SHToC0;
    Trajectory C0ToC1;

    SHToNE = l.trajectories.get("SHToNE");
    NEToSH = l.trajectories.get("NEToHS");
    SHToNE2 = l.trajectories.get("HSToNE2");
    NE2ToSH = l.trajectories.get("NE2ToHS");
    SHToC0 = l.trajectories.get("HSToC0");
    C0ToC1 = l.trajectories.get("C0ToC1");

    t = new TrajectorySet(!isHumanSide, SHToNE, NEToSH, SHToNE2, NE2ToSH, SHToC0, C0ToC1);
  }

  @Override
  public void routine() {
    d.simResetWorldPose(t.initalPose());
    d.zeroGyro(t.initalPose().getRotation().getDegrees());
    sh.followPlan(false);
    sh.setDesiredState(Shooter.State.STOW_HOOD);

    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(new TrajectoryAction(t.next(), d));
    su.mIntake.stateRequest(Intake.State.IDLE).act();

    r(new TrajectoryAction(t.next(), d));
    r(new ShootAction(5, su,1));
    
    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(new TrajectoryAction(t.next(), d));
    su.mIntake.stateRequest(Intake.State.IDLE).act();

    r(new TrajectoryAction(t.next(), d));
    r(new ShootAction(5, su, 1));


    if (climbSelection == ClimbSelection.SHOULD_CLIMB) {
        r(new TrajectoryAction(t.next(), d));
      r(new ClimbAction(c));
      r(new TrajectoryAction(t.next(), d));

      r(new ClimbAction(c));
    }
  }
}

