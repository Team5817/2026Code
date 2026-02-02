package com.team5817.lib.requests;

import com.team5817.frc2026.planners.ShootingPlanner;
import com.team5817.frc2026.subsystems.Superstructure;
import org.littletonrobotics.junction.Logger;

public class AutoShootRequest extends Request {
  Superstructure s;

  public AutoShootRequest(Superstructure s) {
    this.s = s;
  }

  @Override
  public void update() {
    s.mShooter.followPlan(true);
    Logger.recordOutput("Shooter/Should Shoot", ShootingPlanner.shouldShoot());
    // if(planner.shouldShoot())
    // 	s.mIndexer.setDesiredState(Superstructure.IndexerState.INDEX);
    // else
    // 	s.mIndexer.setDesiredState(Superstructure.IndexerState.OFF);
  }

  @Override
  public void cleanup() {
    s.mShooter.followPlan(false);
  }

  @Override
  public boolean isFinished() {
    return false; // runs until interrupted
  }

  @Override
  public void act() {}
}
