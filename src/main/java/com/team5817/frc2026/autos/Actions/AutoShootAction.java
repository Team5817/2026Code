package com.team5817.frc2026.autos.Actions;

import com.team5817.frc2026.planners.ShootingPlanner;
import com.team5817.frc2026.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class AutoShootAction implements Action {
  Timer timer;
  double durationSeconds;

  public AutoShootAction(double durationSeconds, ShootingPlanner p, Superstructure s) {
    this.durationSeconds = durationSeconds;
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
    // indexer off
  }

  @Override
  public void start() {
    timer.reset();
    timer.start();
    // indexer on
  }
}
