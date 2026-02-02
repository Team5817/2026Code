package com.team5817.frc2026.autos.Modes;

import com.team5817.frc2026.autos.Actions.AutoShootAction;
import com.team5817.frc2026.autos.Actions.ClimbAction;
import com.team5817.frc2026.autos.Actions.TrajectoryAction;
import com.team5817.frc2026.autos.Actions.WaitAction;
import com.team5817.frc2026.autos.AutoBase;
import com.team5817.frc2026.autos.AutoModeFactory.ClimbSelection;
import com.team5817.frc2026.autos.TrajectoryLibrary.l;
import com.team5817.frc2026.subsystems.Climb.Climb;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;

public class NRHT extends AutoBase {

  private Drive d;
  private Superstructure su;
  private TrajectorySet t;
  private Climb c;
  private Shooter sh;
  private ClimbSelection climbSelection;

  public NRHT(Superstructure s, ClimbSelection climbSelection) {
    this.d = s.mDrive;
    this.su = s;
    this.sh = s.mShooter;
    
    this.c = s.mClimb;
    this.climbSelection = climbSelection;

    Trajectory SHTToN1;
    Trajectory N1ToH;
    Trajectory HToC0;

    SHTToN1 = l.trajectories.get("SHTToN1");
    N1ToH = l.trajectories.get("N1ToH");
    HToC0 = l.trajectories.get("HToC0");

    t = new TrajectorySet(false, SHTToN1, N1ToH, HToC0);
  }

  @Override
  public void routine() {
    d.simResetWorldPose(t.initalPose());

    r(new AutoShootAction(3, su));
    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(new TrajectoryAction(t.next(), d));
    r(new TrajectoryAction(t.next(), d));
    r(new WaitAction(5.0));
    r(new AutoShootAction(7.0, su));
    r(new TrajectoryAction(t.next(), d));

    if (climbSelection == ClimbSelection.SHOULD_CLIMB) {
      r(new ClimbAction(c));
    }

    sh.followPlan(false);
  }
}
