package com.team5817.frc2026.Controls;

import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.lib.requests.AutoShootRequest;
import com.team5817.lib.requests.EmptyRequest;


public class DriverControls {

  public ControlBoard mControlBoard;
  Superstructure s;
  Drive d;

  public DriverControls(Drive d, Superstructure s) {
    this.d = d;
    this.s = s;
    this.mControlBoard = new ControlBoard(d);
    this.driver = mControlBoard.driver;
  }

  public void oneControllerMode() {
   
    if (driver.getBackButton()) {
      s.mShooter.setDesiredState(Shooter.State.STOW);
      s.mIntake.conformToState(Intake.State.STOW);
      return;
    }

    s.mShooter
        .getPlanner()
        .setOverride(driver.getRightTriggerAxis() >0.2);

    // Zero
    if (driver.getStartButton()) d.allianceZeroGyro();

    // LT Intake
    if (driver.leftTrigger.wasActivated()) {
      s.mIntake.conformToState(Intake.State.INTAKING);
    }
    if (driver.leftTrigger.wasReleased()) {
      s.mIntake.conformToState(Intake.State.IDLE);
    }
    if (driver.rightBumper.wasReleased() && s.mIntake.getMState() == Intake.State.SQUEEZING)
      s.mIntake.conformToState(Intake.State.IDLE);

    // MANUAL TURRET
    if(driver.rightBumper.wasActivated()) {
      s.mShooter.getPlanner().changeTurretBy(20);
    }
    if(driver.leftBumper.wasActivated()) {
      s.mShooter.getPlanner().changeTurretBy(-20);
    }

    //MANUAL HOOD
    if (driver.POV0.wasActivated()) {
      s.mShooter.getPlanner().changeHoodBy(2);
    } else if (driver.POV180.wasActivated()) {
      s.mShooter.getPlanner().changeHoodBy(-2);
    }

    //MANUAL FLYWHEEL
    if(driver.POV90.wasActivated()) {
      s.mShooter.getPlanner().changeFlywheelBy(20);
    } else if (driver.POV270.wasActivated()) {
      s.mShooter.getPlanner().changeFlywheelBy(-20);
    }

    //MANUAL TURRET FAST
    if(driver.getXButtonPressed()) {
      s.mShooter.getPlanner().changeTurretBy(90);
    }
    if(driver.getBButtonPressed()) {
      s.mShooter.getPlanner().changeTurretBy(-90);
    }

    // Y Close
    if (driver.getYButtonPressed()) {
      s.request(s.CloseShotRequest());
    }
    if (driver.getYButtonReleased()) {
      s.request(new AutoShootRequest(s.mShooter.getPlanner(), s).addName("AutoShoot"));
    }

    // A Force Hood + Stow Intake
    if (driver.getAButtonPressed()) {
      s.mShooter.forceStow(true);
      s.mIntake.conformToState(Intake.State.STOW);
      s.request(new EmptyRequest());
    }
    if (driver.getAButtonReleased()) {
      s.mShooter.forceStow(false);
      s.request(new AutoShootRequest(s.mShooter.getPlanner(), s).addName("AutoShoot"));
    }

  }

  CustomXboxController driver;

}
