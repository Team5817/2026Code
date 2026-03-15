package com.team5817.frc2026.autos.Modes;

import com.team5817.frc2026.autos.Actions.ClimbAction;
import com.team5817.frc2026.autos.Actions.ParallelAction;
import com.team5817.frc2026.autos.Actions.ShootAction;
import com.team5817.frc2026.autos.Actions.ShootWhenInZone;
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
import java.util.List;

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
    Trajectory TelePrep;
    Trajectory ClimbPrep;
    Trajectory ClimbEntry;

    Intake1 = l.trajectories.get("SHToNE");
    Return1 = l.trajectories.get("NEToHS");

    if (isClose) {
      Intake2 = l.trajectories.get("CHSToNE2");
      if (endSelection == EndSelection.HUMAN) {
        Return2 = l.trajectories.get("NE2ToH");
      } else {
        Return2 = l.trajectories.get("CNE2ToHS");
      }
    } else {
      Intake2 = l.trajectories.get("HSToNE2");
      if (endSelection == EndSelection.HUMAN) {
        Return2 = l.trajectories.get("NE2ToH");
      } else {
        Return2 = l.trajectories.get("NE2ToHS");
      }
    }

    // Climb Option
    ClimbPrep = l.trajectories.get("HSToC0");
    ClimbEntry = l.trajectories.get("C0ToC1");
    TelePrep = l.trajectories.get("HSToNE2");

    if (endSelection == EndSelection.SHOULD_NOT_CLIMB) {
      t = new TrajectorySet(!isHumanSide, Intake1, Return1, Intake2, Return2, TelePrep);
    } else {
      t =
          new TrajectorySet(
              !isHumanSide, Intake1, Return1, Intake2, Return2, TelePrep, ClimbPrep, ClimbEntry);
    }
  }

  @Override
  public void routine() {
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

    r(new ShootAction(6, su, 1));

    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(new TrajectoryAction(t.next(), 1, d));

    switch (endSelection) {
      case HUMAN:
        r(
            new ParallelAction(
                List.of(new TrajectoryAction(t.next(), 1.5, d), new ShootWhenInZone(100, su, 1))));
        break;
      case SHOULD_CLIMB:
        r(new TrajectoryAction(t.next(), 1.5, d));
        su.mIntake.stateRequest(Intake.State.IDLE).act();

        r(new ParallelAction(List.of(new TrajectoryAction(t.next(), 1.5, d), new ClimbAction(c))));
        r(new ShootAction(4, su, 1));

        r(new TrajectoryAction(t.next(), 1, d));
        r(new ClimbAction(c));
        break;
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
