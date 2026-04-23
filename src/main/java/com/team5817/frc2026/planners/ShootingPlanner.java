package com.team5817.frc2026.planners;


import com.team5817.frc2026.subsystems.Shooter.Shooter.State;
import lombok.Getter;
import lombok.Setter;

import java.util.function.DoubleSupplier;


public class ShootingPlanner {

  @Setter
  @Getter
  boolean override = false; 
  @Setter
  @Getter
  double manualTurret = 0;
  @Setter
  @Getter
  double manualHood = 0;
  @Setter
  @Getter
  double manualFlywheel = 0;



  public DoubleSupplier getTurretAngleSupplier(ShootingTarget hub) {
      return this::getManualTurret;
  }
  public void changeTurretBy(double deg){
    setManualTurret(getManualTurret()+deg);
  }

  public DoubleSupplier getHoodAngleSupplier(ShootingTarget hub) {
      return this::getManualHood;
  }

  public void changeHoodBy(double deg){
    setManualHood(getManualHood()+deg);
  } 

  public DoubleSupplier getFlywheelSpeedSupplier(ShootingTarget hub) {
      return this::getManualFlywheel;
  }

  public void changeFlywheelBy(double rpm){
    setManualFlywheel(getManualFlywheel()+rpm);
  } 

  public boolean shouldShoot() {
      return override;
  }

  public State recommendedShooterState() {

  return State.HUB;
  }


}
