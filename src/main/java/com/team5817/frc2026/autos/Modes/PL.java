package com.team5817.frc2026.autos.Modes;

import com.team5817.frc2026.autos.Actions.ClimbAction;
import com.team5817.frc2026.autos.Actions.ParallelAction;
import com.team5817.frc2026.autos.Actions.ShootAction;
import com.team5817.frc2026.autos.Actions.TrajectoryAction;
import com.team5817.frc2026.autos.AutoBase;
import com.team5817.frc2026.autos.AutoModeFactory.EndSelection;
import com.team5817.frc2026.autos.TrajectoryLibrary.l;
import com.team5817.frc2026.planners.ShootingPlanner;
import com.team5817.frc2026.subsystems.Climb.Climb;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;
import java.util.List;

public class PL extends AutoBase {
  private Drive d;
  private Superstructure su;
  private TrajectorySet t;
  private Climb c;
  private Shooter sh;
  private EndSelection climbSelection;
  private ShootingPlanner p;

  public PL(Superstructure s, EndSelection climbSelection) {
    this.d = s.mDrive;
    this.su = s;
    this.sh = s.mShooter;
    this.p = sh.getPlanner();
    this.c = s.mClimb;
    this.climbSelection = climbSelection;

    Trajectory SCToCO = l.trajectories.get("SCToCO");
    Trajectory COToC0 = l.trajectories.get("COToC0");
    Trajectory C0ToC1 = l.trajectories.get("C0ToC1");

    t = new TrajectorySet(false, SCToCO, COToC0, C0ToC1);
  }

  @Override
  public void routine() {

    d.simResetWorldPose(t.initalPose());
    sh.followPlan(false);
    d.zeroGyro(t.initalPose().getRotation().getDegrees());

    r(new TrajectoryAction(t.next(), d));
    r(new ShootAction(4, su));

    if (climbSelection == EndSelection.SHOULD_CLIMB) {
      r(new ParallelAction(List.of(new TrajectoryAction(t.next(), d), new ClimbAction(c))));
      r(new TrajectoryAction(t.next(), d));
      r(new ClimbAction(c));
    }
  }
}
