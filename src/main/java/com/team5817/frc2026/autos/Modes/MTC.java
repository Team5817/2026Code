package com.team5817.frc2026.autos.Modes;

import com.team5817.frc2026.autos.Actions.ShootAction;
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

public class MTC extends AutoBase {
  private Drive d;
  private Superstructure su;
  private TrajectorySet t;
  private Shooter sh;
  private ShootingPlanner p;

  public MTC(Superstructure s, boolean isHumanSide, boolean isClose) {
    this.d = s.mDrive;
    this.su = s;
    this.sh = s.mShooter;
    this.p = sh.getPlanner();

    Trajectory Intake1;
    Trajectory ReturnShoot1;

    Intake1 = l.trajectories.get("HSToCNE2");
    ReturnShoot1 = l.trajectories.get("CNE2ToNSHOT");
    
    t = new TrajectorySet(!isHumanSide, Intake1, ReturnShoot1);
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

    r(new ShootAction(3, su, 1.5));

    su.mIntake.stateRequest(Intake.State.INTAKING).act();
    r(new TrajectoryAction(t.next(), -.3, d));

    r(new TrajectoryAction(t.next(), 1.0, d));
    su.mIntake.stateRequest(Intake.State.IDLE).act();
    r(new ShootAction(8, su, 3));
  }
}
