package com.team5817.frc2026.Controls;

import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Indexer.Indexer;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Shield.Shield;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.lib.requests.AutoShootRequest;
import com.team5817.lib.requests.EmptyRequest;

/**
 * The DriverControls class handles the input from the driver and co-driver controllers and
 * translates them into actions for the robot's subsystems.
 */
public class DriverControls {

  public ControlBoard mControlBoard;
  Superstructure s;
  Drive d;

  public DriverControls(Drive d, Superstructure s) {
    this.d = d;
    this.s = s;
    this.mControlBoard = new ControlBoard(d);
    this.driver = mControlBoard.driver;
    this.codriver = mControlBoard.operator;
  }

  public void oneControllerMode() {
    if (driver.getBackButton()) {
      s.mShooter.setDesiredState(Shooter.State.STOW);
      s.mIntake.conformToState(Intake.State.STOW);
      return;
    }

    s.mShooter
        .getPlanner()
        .setOverride(driver.getRightBumperButton() || codriver.getLeftTriggerAxis() > 0.2);

    // RB don't Shoot
    s.setAllowAutoShoot(!driver.getRightBumperButton());
    if (driver.getStartButton()) d.allianceZeroGyro();

    // LT Intake
    if (driver.leftTrigger.wasActivated()) {
      s.mIntake.conformToState(Intake.State.INTAKING);
    }
    if (codriver.rightBumper.wasActivated() && !driver.leftTrigger.isBeingPressed()) {
      s.mIntake.conformToState(Intake.State.SQUEEZING);
    }
    if (driver.leftTrigger.wasReleased()) {
      s.mIntake.conformToState(Intake.State.IDLE);
    }
    if (driver.rightBumper.wasReleased() && s.mIntake.getMState() == Intake.State.SQUEEZING)
      s.mIntake.conformToState(Intake.State.IDLE);

    // LB Outtake
    if (driver.leftBumper.wasActivated()) {
      s.mIntake.conformToState(Intake.State.EXHAUSTING);
    }
    if (driver.leftBumper.wasReleased() && s.mIntake.getMState() == Intake.State.EXHAUSTING) {
      s.mIntake.conformToState(Intake.State.IDLE);
    }

    if (codriver.leftBumper.wasActivated()) {
      s.mIndexer.stateRequest(Indexer.State.EXHAUST);
    }
    if (codriver.leftBumper.wasReleased() && s.mIndexer.getState() == Indexer.State.EXHAUST) {
      s.mIndexer.stateRequest(Indexer.State.IDLE);
    }
    if (codriver.aButton.wasActivated()) {
      s.mIntake.home();
    }

    // RT Slow mode
    double scalar =
        (driver.getRightBumperButton() || codriver.getLeftTriggerAxis() > 0.2) ? 0.7 : 1.0;
    mControlBoard.setSwerveScalar(scalar);
    d.setSpeedScalar(scalar); // TODO integrate to sotm to reduce sporadicitiy

    // Y Close
    if (driver.getYButtonPressed()) {
      s.request(s.CloseShotRequest());
    }
    if (driver.getYButtonReleased()) {
      s.request(new AutoShootRequest(s.mShooter.getPlanner(), s).addName("AutoShoot"));
    }

    // Intake Stow
    if (driver.getAButtonPressed()) {
      s.mIntake.conformToState(Intake.State.STOW);
    }

    // B Force Hood
    if (driver.getBButtonPressed() || codriver.getBButtonPressed()) {
      s.mShooter.forceStow(true);
      s.mIndexer.setState(Indexer.State.IDLE);
      s.request(new EmptyRequest());
    }
    if (driver.getBButtonReleased() || codriver.getBButtonReleased()) {
      s.mShooter.forceStow(false);
      s.request(new AutoShootRequest(s.mShooter.getPlanner(), s).addName("AutoShoot"));
    }

    // Shield Extend 
    if (driver.getXButtonPressed()) {
      if(s.mShield.getDesiredState() == Shield.State.EXTENDED || s.mIntake.getMState() == Intake.State.STOW)
          s.mShield.advanceShieldRequest().act();
    }

    // Shield Zero
    if (driver.getPOV() == 180) {
      s.mShield.zeroRequest().act();
    }
  }

  CustomXboxController driver;
  CustomXboxController codriver;

  /**
   * Handles the input for the two controller mode. This mode is used when both driver and co-driver
   * controllers are available.
   */
  public void twoControllerMode() {
    if (driver.getStartButton()) {
      d.allianceZeroGyro();
    }
  }
}
