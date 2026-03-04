package com.team5817.frc2026.autos.Actions;

import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Spindexer.Indexer;
import com.team5817.frc2026.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class ShootAction implements Action {
  Timer timer;
  double durationSeconds;
  Superstructure s;

  public ShootAction(double durationSeconds, Superstructure s) {
    this.durationSeconds = durationSeconds;
    this.s = s;
    timer = new Timer();
  }

  @Override
  public boolean isFinished() {
    Logger.recordOutput("Auto/Timer", timer.get());
    return timer.get() > durationSeconds;
  }

  @Override
  public void update() {}

  @Override
  public void done() {
    s.mIndexer.setState(Indexer.State.IDLE);
    s.mShooter.setDesiredState(Shooter.State.STOW);
  }

  @Override
  public void start() {
    s.mShooter.followPlan(false);
    timer.reset();
    timer.start();
    s.mIndexer.setState(Indexer.State.FEED);
    s.mShooter.setDesiredState(Shooter.State.HUB);
  }
}
