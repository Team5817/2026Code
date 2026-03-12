package com.team5817.frc2026.autos.Actions;

import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Spindexer.Indexer;
import com.team5817.frc2026.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class ShootAction implements Action {
  Timer timer;
  Timer spinupTimer;
  double durationSeconds;
  double spinupTime;
  Superstructure s;

  public ShootAction(double durationSeconds, Superstructure s) {
    this(durationSeconds, s, 0);
  }

  public ShootAction(double durationSeconds, Superstructure s, double spinupTime) {
    this.spinupTime = spinupTime;
    this.durationSeconds = durationSeconds;
    this.s = s;
    timer = new Timer();
    spinupTimer = new Timer();
  }

  @Override
  public boolean isFinished() {
    Logger.recordOutput("Auto/Timer", timer.get());
    return timer.get() > durationSeconds;
  }

  @Override
  public void update() {
    Logger.recordOutput("SpinTIMER", spinupTimer.get());
    if (spinupTimer.get() > spinupTime) {
      s.mIndexer.setState(Indexer.State.FEED);
    } else {
      s.mIndexer.setState(Indexer.State.SPINUP);
    }
  }

  @Override
  public void done() {
    s.mIndexer.setState(Indexer.State.IDLE);
    s.mIntake.conformToState(Intake.State.IDLE);
    s.mShooter.setDesiredState(Shooter.State.STOW_HOOD);
    s.mShooter.forceStow(true);
  }

  @Override
  public void start() {
    s.mShooter.followPlan(false);
    timer.reset();
    timer.start();
    spinupTimer.reset();
    spinupTimer.start();
    s.mIntake.conformToState(Intake.State.AGITATE);
    s.mIndexer.setState(Indexer.State.SPINUP);
    s.mShooter.setDesiredState(Shooter.State.HUB);
    s.mShooter.forceStow(false);
  }
}
