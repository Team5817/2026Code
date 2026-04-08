package com.team5817.frc2026.autos.Modes;

import com.team5817.frc2026.autos.Actions.ShootAction;
import com.team5817.frc2026.autos.Actions.TrajectoryAction;
import com.team5817.frc2026.autos.AutoBase;
import com.team5817.frc2026.autos.TrajectoryLibrary.l;
import com.team5817.frc2026.planners.ShootingPlanner;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Elevator.Elevator;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;

public class MT extends AutoBase {
  private Drive d;
  private Superstructure su;
  private TrajectorySet t;
  private Elevator e;
  private Shooter sh;
  private ShootingPlanner p;

  public MT(Superstructure s, boolean isHumanSide, boolean isClose) {
    this.d = s.mDrive;
    this.su = s;
    this.sh = s.mShooter;
    this.p = sh.getPlanner();
    this.e = s.mElevator;

    Trajectory Intake1;
    Trajectory Return1;
    Trajectory Intake2;
    Trajectory Return2;
    Trajectory ReturnShoot2;
    Trajectory TelePrep;

    Intake1 = l.trajectories.get("SHToMT1");
    Return1 = l.trajectories.get("MT1ToHS");
    ReturnShoot2 = l.trajectories.get("CNE2ToNSHOT");

    if (isClose) {
      Intake2 = l.trajectories.get("HSToCNE2");
      {
        Return2 = l.trajectories.get("CNE2ToNSHOT");
      }

    } else {
      Intake2 = l.trajectories.get("HSToNE2");
      {
        Return2 = l.trajectories.get("NE2ToHS");
      }
    }

    TelePrep = l.trajectories.get("NSHOTToNE2");

    t = new TrajectorySet(!isHumanSide, Intake1, Return1, Intake2, Return2, TelePrep);
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

    r(new ShootAction(4, su, 2));

    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(new TrajectoryAction(t.next(), -.3, d));

    r(new TrajectoryAction(t.next(), 1.5, d));
    su.mIntake.stateRequest(Intake.State.IDLE).act();
    r(new ShootAction(10, su, 2));
  }
}
