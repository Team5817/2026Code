package com.team5817.frc2026.autos.Modes;

import com.team5817.frc2026.autos.Actions.ShootAction;
import com.team5817.frc2026.autos.Actions.TrajectoryAction;
import com.team5817.frc2026.autos.AutoBase;
import com.team5817.frc2026.autos.TrajectoryLibrary.l;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;

public class PL extends AutoBase {
  private Drive d;
  private Superstructure su;
  private TrajectorySet t;
  private Shooter sh;

  public PL(Superstructure s) {
    this.d = s.mDrive;
    this.su = s;
    this.sh = s.mShooter;

    Trajectory SCToCO = l.trajectories.get("SCToCO");
    t = new TrajectorySet(false, SCToCO);
  }

  @Override
  public void routine() {
    d.simResetWorldPose(t.initalPose());
    sh.followPlan(false);
    d.zeroGyro(t.initalPose().getRotation().getDegrees());

    r(new TrajectoryAction(t.next(), d));
    r(new ShootAction(4, su));
  }
}
