package com.team5817.frc2026.autos.Actions;

import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.frc2026.subsystems.Spindexer.SpindexerGroup.State;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class AutoShootAction implements Action {
  Timer timer;
  double durationSeconds;
  Superstructure s;

  public AutoShootAction(double durationSeconds , Superstructure s) {
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
    s.mSpindexerGroup.setState(State.IDLE);
  }

  @Override
  public void start() {
    timer.reset();
    timer.start();
    s.mSpindexerGroup.setState(State.FEED_TURRET);
  }
}
