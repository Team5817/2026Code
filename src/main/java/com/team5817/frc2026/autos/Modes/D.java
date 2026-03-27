package com.team5817.frc2026.autos.Modes;

import com.team5817.frc2026.autos.Actions.ParallelAction;
import com.team5817.frc2026.autos.Actions.ShootAction;
import com.team5817.frc2026.autos.Actions.ShootWhenInZone;
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
import java.util.List;

public class D extends AutoBase {
  private Drive d;
  private Superstructure su;
  private TrajectorySet t;
  private Shooter sh;
  private ShootingPlanner p;

  public D(Superstructure s) {
    this.d = s.mDrive;
    this.su = s;
    this.sh = s.mShooter;
    this.p = sh.getPlanner();

    Trajectory SDToMT2 = l.trajectories.get("SDToMT2");
    Trajectory MT2ToSDR = l.trajectories.get("MT2ToSDR");
    Trajectory SDRToDP = l.trajectories.get("SDRToDP");
    Trajectory DPToDE = l.trajectories.get("DPToDE");
    t = new TrajectorySet(SDToMT2, MT2ToSDR, SDRToDP, DPToDE);
  }

  @Override
  public void routine() {
    d.simResetWorldPose(t.initalPose());
    d.zeroGyro(t.initalPose().getRotation().getDegrees());
    sh.followPlan(false);
    sh.setDesiredState(Shooter.State.STOW_HOOD);
    su.mIntake.stateRequest(Intake.State.STOW).act();
    p.setOverride(false);
    sh.forceStow(true);

    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(new TrajectoryAction(t.next(), 1, d));
    r(new TrajectoryAction(t.next(), -.1, d));

    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(
        new ParallelAction(
            List.of(new TrajectoryAction(t.next(), d), new ShootWhenInZone(5, su, 60))));

    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(new TrajectoryAction(t.next(), 1, d));
    r(new ShootAction(6, su, 2));
  }
}
