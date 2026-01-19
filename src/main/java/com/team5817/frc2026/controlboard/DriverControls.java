package com.team5817.frc2026.controlboard;

import com.team5817.frc2026.ActiveTracker;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Shooter.Shooter;

/**
 * The DriverControls class handles the input from the driver and co-driver
 * controllers
 * and translates them into actions for the robot's subsystems.
 */
public class DriverControls {

  public ControlBoard mControlBoard;

  Superstructure s;
  Drive d;

  /**
   * Constructor for the DriverControls class.
   * Initializes the Drive and Superstructure instances and sets the initial goal
   * state.
   */
  public DriverControls(Drive d, Superstructure s) {
    this.d = d;
    this.s = s;
    this.mControlBoard = new ControlBoard(d);
    this.driver = mControlBoard.driver;
    this.codriver = mControlBoard.operator;
  }

  /* ONE CONTROLLER */

  /**
   * Handles the input for the one controller mode.
   * This mode is used when only one controller is available for the driver.
   */
  public void oneControllerMode() {
    // mDrive.overrideHeading(true);
    if (driver.getStartButton())
      d.allianceZeroGyro();

    //LT intake
    if(driver.leftTrigger.wasActivated()){
      s.mIntake.conformToState(Intake.State.INTAKING);
    }
    if(driver.leftTrigger.wasReleased()){
      s.mIntake.conformToState(Intake.State.IDLE);
    }
    //LB Reverse Indexer
    if(driver.getLeftBumperButton()){
      s.mIndexer.conformToState(Superstructure.IndexerState.REVERSE);
    }
    //RB don't Shoot
    s.setAllowAutoShoot(!driver.getRightBumperButton());
    //RT Slow mode
    mControlBoard.setSwerveScalar(1-driver.getRightTriggerAxis()*.7);//coefficient is percent to reduce speed by
    
    //Y Close
    if(driver.getYButtonPressed()){
      s.request(s.CloseShotRequest());
    }
    if(driver.getYButtonReleased()){
      s.clearRequestQueue();
    }
    //A Far
    if(driver.getAButtonPressed()){
      s.request(s.FarShotRequest());
    }
    if(driver.getAButtonReleased()){
      s.clearRequestQueue();
    }
    //B Force Hood
    if(driver.getBButtonPressed()){
      s.mShooter.forceStow(true);
    }
    if(driver.getBButtonReleased()){
      s.mShooter.forceStow(false);
    }
    //Down Climb
    if(driver.POV180.wasActivated()){
      s.request(s.advanceClimbRequest());
    }
    //Up Unclimb
    if(driver.POV0.wasActivated()){
      s.request(s.resetClimb());
    }
    //Controller Shake
    if(ActiveTracker.shouldShakeController()){
      driver.rumble(0.3, 1);;
    }
  }

  CustomXboxController driver;
  CustomXboxController codriver;
  /* TWO CONTROLLERS */
  double lastTime = 0;

  /**
   * Handles the input for the two controller mode.
   * This mode is used when both driver and co-driver controllers are available.
   */
  public void twoControllerMode() {
    if (driver.getStartButton()){
      d.allianceZeroGyro();
    }
  }
}
