package com.team5817.frc2026.autos.Actions;

import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.frc2026.subsystems.Shooter.Shooter;

public class ShootWhenInZone implements Action {

  private ShootAction shootAction;
  private Superstructure s;
  private boolean hasStarted = false;

  public ShootWhenInZone(double durationSeconds, Superstructure s, double spinUpTime) {
    this.shootAction = new ShootAction(durationSeconds, s,spinUpTime);
    this.s = s;
  }

  @Override
  public void start() {
  }

  @Override
  public void update() {
    if (!hasStarted && s.mShooter.getPlanner().recommendedShooterState() == Shooter.State.HUB) {
      shootAction.start();
      hasStarted = true;
    }
    if(hasStarted)
        shootAction.update();
  }

  @Override
  public boolean isFinished() {
    return shootAction.isFinished() && hasStarted;
  }

   @Override
   public void done() {
    shootAction.done();
   }
    
}
