package com.team5817.frc2026.Controls;

import org.littletonrobotics.junction.Logger;

import com.team5817.frc2026.ActiveTracker;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Spindexer.SpindexerGroup;
import com.team5817.frc2026.subsystems.Stationary.FixedShooter;
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

    // LB Dump
    if (driver.getLeftBumperButtonPressed()) {
      s.mFixedShooter.setDesiredState(FixedShooter.State.HUB);
      s.mSpindexerGroup.setState(SpindexerGroup.State.FEED_BOTH);
    }

    if (driver.getLeftBumperButtonReleased()) {
      s.mSpindexerGroup.setState(SpindexerGroup.State.IDLE);
      s.mIntake.conformToState(Intake.State.IDLE);
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

    // Climb Down
    if (driver.POV180.wasActivated()) {
      s.mClimb.advanceClimbRequest().act();
    }

    // Climb Zero
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
