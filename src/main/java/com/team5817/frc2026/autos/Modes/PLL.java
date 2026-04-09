package com.team5817.frc2026.autos.Modes;

import com.team5817.frc2026.autos.Actions.ParallelAction;
import com.team5817.frc2026.autos.Actions.ShootAction;
import com.team5817.frc2026.autos.Actions.TrajectoryAction;
import com.team5817.frc2026.autos.AutoBase;
import com.team5817.frc2026.autos.TrajectoryLibrary.l;
import com.team5817.frc2026.planners.ShootingPlanner;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Shield.Shield;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;
import java.util.List;

public class PLL extends AutoBase {
  private Drive d;
  private Superstructure su;
  private TrajectorySet t;
  private Shield c;
  private Shooter sh;
  private ShootingPlanner p;

  public PLL(Superstructure s) {
    this.d = s.mDrive;
    this.su = s;
    this.sh = s.mShooter;
    this.p = sh.getPlanner();
    this.c = s.mShield;

    Trajectory SCToCO = l.trajectories.get("SCToCO");
    Trajectory COToC02 = l.trajectories.get("COToC02");
    Trajectory C02ToC2 = l.trajectories.get("C02ToC2");

    t = new TrajectorySet(false, SCToCO, COToC02, C02ToC2);
  }

  @Override
  public void routine() {

    d.simResetWorldPose(t.initalPose());
    sh.followPlan(false);
    d.zeroGyro(t.initalPose().getRotation().getDegrees());

    r(new TrajectoryAction(t.next(), d));
    r(new ShootAction(4, su));

    r(new ParallelAction(List.of(new TrajectoryAction(t.next(), d) )));
    r(new TrajectoryAction(t.next(), d));
  }
}
