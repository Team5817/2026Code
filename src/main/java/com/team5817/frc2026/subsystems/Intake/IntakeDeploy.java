package com.team5817.frc2026.subsystems.Intake;

import com.team5817.lib.drivers.Servos.ServoConstants;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;
import lombok.Getter;

/** The IntakeDeploy class controls the deployment mechanism of the intake system. */
public class IntakeDeploy extends StateBasedServoMotorSubsystem<IntakeDeploy.State> {

  /**
   * Constructs a new IntakeDeploy subsystem.
   *
   * @param constants The constants for the servo motor subsystem.
   * @param encoder_constants The constants for the absolute encoder.
   */
  public IntakeDeploy(final ServoConstants constants, ServoMotorIO io) {
    super(IntakeDeploy.State.IDLE, io, true);
  }

  /** Represents the different states of the intake deployment. */
  public enum State implements ServoState {
    IDLE(0.3175), // og .2667
    OUT(0.3175),
    SQUEEZE(0.1), // TODO placeholder
    ZERO(0);

    @Getter private double demand = 0;
    @Getter private double allowableError = 0;
    @Getter private boolean disabled = false;

    /**
     * p Constructs a new State.
     *
     * @param output The output value for the state.
     * @param allowable_error The allowable error for the state.
     */
    State(double output) {
      this.demand = output;
      this.allowableError = .1;
    }

    State() {
      this.disabled = true;
    }

    @Override
    public ControlState getControlState() {
      return ControlState.POSITION;
    }
  }
}
