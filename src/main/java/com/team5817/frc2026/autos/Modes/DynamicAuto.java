package com.team5817.frc2026.autos.Modes;

import com.team5817.frc2026.autos.Actions.ShootAction;
import com.team5817.frc2026.autos.Actions.TrajectoryAction;
import com.team5817.frc2026.autos.AutoBase;
import com.team5817.frc2026.autos.AutoModeFactory.EndSelection;
import com.team5817.frc2026.autos.TrajectoryLibrary.l;
import com.team5817.frc2026.planners.ShootingPlanner;
import com.team5817.frc2026.subsystems.Climb.Climb;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.lib.motion.Trajectory;
import com.team5817.lib.motion.TrajectorySet;
import org.littletonrobotics.junction.Logger;

public class DynamicAuto extends AutoBase {
  private Drive d;
  private Superstructure su;
  private TrajectorySet t;
  private Climb c;
  private Shooter sh;
  private EndSelection endSelection;
  private ShootingPlanner p;

  public DynamicAuto(
      Superstructure s, EndSelection endSelection, boolean isHumanSide, boolean isClose) {
    this.d = s.mDrive;
    this.su = s;
    this.sh = s.mShooter;
    this.p = sh.getPlanner();
    this.c = s.mClimb;
    this.endSelection = endSelection;

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

 if (endSelection == EndSelection.SHOULD_NOT_CLIMB) {
      t = 
          new TrajectorySet(
              !isHumanSide, 
              Intake1, 
              Return1, 
              Intake2, 
              Return2, 
              TelePrep);
    } 
    else {
      t =
          new TrajectorySet(
              !isHumanSide, Intake1, Return1, Intake2, Return2, ReturnShoot2, TelePrep);
    }
  }

  @Override
  public void routine() {

    Logger.recordOutput("Auto/ cached DESIRED", endSelection);

    d.simResetWorldPose(t.initalPose());
    d.zeroGyro(t.initalPose().getRotation().getDegrees());
    sh.followPlan(false);
    sh.setDesiredState(Shooter.State.STOW_HOOD);
    p.setOverride(false);
    sh.forceStow(true);

    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(new TrajectoryAction(t.next(), 1, d));
    r(new TrajectoryAction(t.next(), 1.5, d));
    su.mIntake.stateRequest(Intake.State.IDLE).act();

    r(new ShootAction(4, su, 1));

    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(new TrajectoryAction(t.next(), 1, d));

    switch (endSelection) {
      case SHOULD_NOT_CLIMB:
        r(new TrajectoryAction(t.next(), 1.5, d));
        su.mIntake.stateRequest(Intake.State.IDLE).act();
        r(new ShootAction(4, su, 1));
        su.mIntake.stateRequest(Intake.State.IDLE).act();
        r(new TrajectoryAction(t.next(), 1.5, d));
        break;
    }
  }
}
