package com.team5817.frc2026.autos.Modes;

import com.team5817.frc2026.autos.Actions.ParallelAction;
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

public class MTB extends AutoBase {
  private Drive d;
  private Superstructure su;
  private TrajectorySet t;
  private Shooter sh;
  private ShootingPlanner p;

  public MTB(Superstructure s, boolean isHumanSide, boolean isClose) {
    this.d = s.mDrive;
    this.su = s;
    this.sh = s.mShooter;
    this.p = sh.getPlanner();

    Trajectory Intake1 = l.trajectories.get("SH2ToMT3");
    Trajectory Return1 = l.trajectories.get("MT3ToB1");
    Trajectory Shoot1 = l.trajectories.get("B1ToSH2");
    Trajectory Intake2 = l.trajectories.get("SH2ToMT3");
    Trajectory Return2 = l.trajectories.get("MT3ToB1");
    Trajectory Shoot2 = l.trajectories.get("B1ToSH2");

    t = new TrajectorySet(isHumanSide, Intake1, Return1, Shoot1, Intake2, Return2, Shoot2);
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
    r(new TrajectoryAction(t.next(), 1.5, d));
    su.mIntake.stateRequest(Intake.State.IDLE).act();

    r(
        new ParallelAction(
            List.of(new TrajectoryAction(t.next(), 1.0, d), new ShootWhenInZone(5.0, su, 1.0))));

    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(new TrajectoryAction(t.next(), 1.0, d));
    r(new TrajectoryAction(t.next(), 1.0, d));
    su.mIntake.stateRequest(Intake.State.IDLE).act();

    r(
        new ParallelAction(
            List.of(new TrajectoryAction(t.next(), 1.0, d), new ShootWhenInZone(6.0, su, 1.0))));
  }
}
