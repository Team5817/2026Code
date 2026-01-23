package com.team5817.frc2026.autos.Modes;

import com.team5817.frc2026.autos.Actions.AutoShootAction;
import com.team5817.frc2026.autos.Actions.ClimbAction;
import com.team5817.frc2026.autos.Actions.TrajectoryAction;
import com.team5817.frc2026.autos.AutoBase;
import com.team5817.frc2026.autos.AutoModeFactory.ClimbSelection;
import com.team5817.frc2026.autos.TrajectoryLibrary.l;
import com.team5817.frc2026.planners.ShootingPlannerI;
import com.team5817.frc2026.subsystems.Climb.Climb;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;

public class NR extends AutoBase {

  private Drive d;
  private Superstructure su;
  private TrajectorySet t;
  private Climb c;
  private Shooter sh;
  private ClimbSelection climbSelection;
  private ShootingPlannerI p;

  public NR(Superstructure s, ClimbSelection climbSelection) {
    this.d = s.mDrive;
    this.su = s;
    this.sh = s.mShooter;
    this.p = sh.getPlanner();
    this.c = s.mClimb;
    this.climbSelection = climbSelection;

    Trajectory SHTToN1;
    Trajectory N1ToC0;

    SHTToN1 = l.trajectories.get("SHTToN1");
    N1ToC0 = l.trajectories.get("N1ToC0");

    t = new TrajectorySet(false, SHTToN1, N1ToC0);
  }

  @Override
  public void routine() {
    d.simResetWorldPose(t.initalPose());

    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(new TrajectoryAction(t.next(), d));
    r(new TrajectoryAction(t.next(), d));
    r(new AutoShootAction(5, p, su));

    if (climbSelection == ClimbSelection.SHOULD_CLIMB) {
      r(new ClimbAction(c));
    }

    sh.followPlan(false);
  }
}
