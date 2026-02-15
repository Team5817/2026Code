package com.team5817.frc2026.controlboard;

import com.team5817.frc2026.ActiveTracker;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Spindexer.SpindexerGroup;
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

  /**
   * Constructor for the DriverControls class. Initializes the Drive and Superstructure instances
   * and sets the initial goal state.
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
   * Handles the input for the one controller mode. This mode is used when only one controller is
   * available for the driver.
   */
  public void oneControllerMode() {
    // mDrive.overrideHeading(true);
    if (driver.getStartButton()) d.allianceZeroGyro();

    // LT intake
    if (driver.leftTrigger.wasActivated()) {
      s.mIntake.conformToState(Intake.State.INTAKING);
    }

    if (driver.leftTrigger.wasReleased()) {
      s.mIntake.conformToState(Intake.State.IDLE);
    }

    // LB Reverse Intake & Spindexer
    if (driver.getLeftBumperButtonPressed()) {
      s.mSpindexerGroup.setState(SpindexerGroup.State.EXHAUST);
      s.mIntake.conformToState(Intake.State.EXHAUSTING);
    }

    if (driver.getLeftBumperButtonReleased()) {
      s.mSpindexerGroup.setState(SpindexerGroup.State.IDLE);
      s.mIntake.conformToState(Intake.State.IDLE);
    }

    // //LeftStick Spindexer Feed Shooter
    if (driver.getLeftStickButtonPressed()) {
      s.mSpindexerGroup.setState(SpindexerGroup.State.FEED_SHOOTER);
    }

    if (driver.getLeftStickButtonReleased()) {
      s.mSpindexerGroup.setState(SpindexerGroup.State.IDLE);
    }

    // //RightStick Spindexer Feed Turret
    if (driver.getRightStickButtonPressed()) {
      s.mSpindexerGroup.setState(SpindexerGroup.State.FEED_TURRET);
    }

    if (driver.getRightStickButtonReleased()) {
      s.mSpindexerGroup.setState(SpindexerGroup.State.IDLE);
    }

    // RB don't Shoot
    s.setAllowAutoShoot(!driver.getRightBumperButton());
    // RT Slow mode
    mControlBoard.setSwerveScalar(
        1 - driver.getRightTriggerAxis() * .7); // coefficient is percent to reduce speed by

    // Y Close
    if (driver.getYButtonPressed()) {
      s.request(s.CloseShotRequest());
    }

    if (driver.getYButtonReleased()) {
      s.request(new AutoShootRequest(s.mShooter.getPlanner(), s).addName("AutoShoot"));
    }

    // A Far
    if (driver.getAButtonPressed()) {
      s.request(s.FarShotRequest());
    }

    if (driver.getAButtonReleased()) {
      s.request(new AutoShootRequest(s.mShooter.getPlanner(), s).addName("AutoShoot"));
    }

    // B Force Hood
    if (driver.getBButtonPressed()) {
      s.mShooter.forceStow(true);
      s.request(new EmptyRequest());
    }

    if (driver.getBButtonReleased()) {
      s.mShooter.forceStow(false);
      s.request(new AutoShootRequest(s.mShooter.getPlanner(), s).addName("AutoShoot"));
    }

    // Down Climb
    if (driver.POV180.wasActivated()) {
      s.mClimb.advanceClimbRequest().act();
    }

    // Up Unclimb
    if (driver.POV0.wasActivated()) {
      s.mClimb.resetClimbStages();
    }

    // Controller Shake
    if (ActiveTracker.shouldShakeController()) {
      driver.rumble(0.3, 1);
      ;
    }
  }

  CustomXboxController driver;
  CustomXboxController codriver;
  /* TWO CONTROLLERS */
  double lastTime = 0;

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
