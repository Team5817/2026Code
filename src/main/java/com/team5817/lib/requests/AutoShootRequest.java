package com.team5817.lib.requests;

import com.team5817.frc2026.planners.ShootingPlanner;
import com.team5817.frc2026.subsystems.Spindexer.Indexer;
import com.team5817.frc2026.subsystems.Superstructure;
import org.littletonrobotics.junction.Logger;

public class AutoShootRequest extends Request {
  ShootingPlanner planner;
  Superstructure s;

  public AutoShootRequest(ShootingPlanner planner, Superstructure s) {
    this.planner = planner;
    this.s = s;
  }

  @Override
  public void update() {
    s.mShooter.followPlan(true);
    Logger.recordOutput("Shooter/Should Shoot", planner.shouldShoot());
    if (planner.shouldShoot()) s.mIndexer.setState(Indexer.State.FEED_TURRET);
    else s.mIndexer.setState(Indexer.State.IDLE);
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
