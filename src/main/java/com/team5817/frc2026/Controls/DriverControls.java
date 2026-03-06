package com.team5817.frc2026.Controls;

import com.team5817.frc2026.ActiveTracker;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Intake.Intake;
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
    s.mShooter.getPlanner().setOverride(driver.getRightBumperButton());

    // RB don't Shoot
    s.setAllowAutoShoot(!driver.getRightBumperButton());
    if (driver.getStartButton()) d.allianceZeroGyro();

    // Manual Zero
    if (codriver.getBackButton()) {
      s.mIntake.conformToState(Intake.State.STOW);
      s.mShooter.forceStow(true);
      // s.mClimb.setState(Climb.State.ZERO);
    }

    // LT Intake
    else if (driver.leftTrigger.isBeingPressed()) {
      s.mIntake.conformToState(Intake.State.INTAKING);
    } else if (codriver.rightTrigger.isBeingPressed()) {
      s.mIntake.conformToState(Intake.State.AGITATE);
    }
    if (driver.leftTrigger.wasReleased() && s.mIntake.getMState() == Intake.State.INTAKING) {
      s.mIntake.conformToState(Intake.State.IDLE);
    }
    if (codriver.rightTrigger.wasReleased() && s.mIntake.getMState() == Intake.State.AGITATE)
      s.mIntake.conformToState(Intake.State.IDLE);

    // LB Outtake
    if (driver.leftBumper.isBeingPressed()) {
      s.mIntake.conformToState(Intake.State.EXHAUSTING);
    }
    if (!driver.leftBumper.isBeingPressed() && s.mIntake.getMState() == Intake.State.EXHAUSTING) {
      s.mIntake.conformToState(Intake.State.IDLE);
    }

    // RT Slow mode
    double scalar = 1 - driver.getRightTriggerAxis() * 0.5;
    mControlBoard.setSwerveScalar(scalar);
    d.setSpeedScalar(scalar);

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

    // B Force Hood (Both)
    if (driver.getBButtonPressed() || codriver.getBButtonPressed()) {
      s.mShooter.forceStow(true);
      s.request(new EmptyRequest());
    }
    if (driver.getBButtonReleased() || codriver.getBButtonReleased()) {
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
