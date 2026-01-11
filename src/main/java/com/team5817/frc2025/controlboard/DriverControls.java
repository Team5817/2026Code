package com.team5817.frc2025.controlboard;

import com.team5817.frc2025.subsystems.Superstructure;
import com.team5817.frc2025.subsystems.Drive.Drive;

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
