package com.team5817.frc2026.autos.Modes;

import com.team5817.frc2026.autos.Actions.ClimbAction;
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

public class DH extends AutoBase {
  private Drive d;
  private Superstructure su;
  private TrajectorySet t;
  private Climb c;
  private Shooter sh;
  private ClimbSelection climbSelection;
  private ShootingPlanner p;

  public DH(Superstructure s, ClimbSelection climbSelection) {
    this.d = s.mDrive;
    this.su = s;
    this.sh = s.mShooter;
    this.p = sh.getPlanner();
    this.c = s.mClimb;
    this.climbSelection = climbSelection;

    Trajectory SDFToDT;
    Trajectory DTToH;
    Trajectory HToC0;
    Trajectory C0ToC1;

    SDFToDT = l.trajectories.get("SDFToDT");
    DTToH = l.trajectories.get("DTToH");
    HToC0 = l.trajectories.get("HToC0");
    C0ToC1 = l.trajectories.get("C0ToC1");

    t = new TrajectorySet(false, SDFToDT, DTToH, HToC0, C0ToC1);
  }

  @Override
  public void routine() {
    sh.followPlan(false);
    d.simResetWorldPose(t.initalPose());
    sh.setDesiredState(Shooter.State.STOW);

    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(new TrajectoryAction(t.next(), d));
    su.mIntake.stateRequest(Intake.State.IDLE).act();

    r(new TrajectoryAction(t.next(), d));
    r(new ShootAction(7, su));
    if (climbSelection == ClimbSelection.SHOULD_CLIMB) {
      r(new TrajectoryAction(t.next(), d));
      r(new TrajectoryAction(t.next(), d));
      r(new ClimbAction(c));
    }
  }
}
