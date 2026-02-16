package com.team5817.frc2026.subsystems.Climb;

import com.team5817.frc2026.RobotVisualizer;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;
import com.team5817.lib.requests.Request;
import com.team5817.lib.requests.SequentialRequest;
import com.team5817.lib.requests.WaitRequest;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lombok.Getter;

/** Elevator subsystem for controlling the elevator mechanism. */
public class Climb extends StateBasedServoMotorSubsystem<Climb.State> {

  /**
   * Constructs an Elevator with the given constants.
   *
   * @param constants the constants for the elevator
   */
  public Climb(ServoMotorIO io) {
    super(State.ZERO, io, true);
  }

  /** Enum representing the different states of the elevator. */
  public enum State implements ServoState {
    ZERO(0),
    READY(140),
    RETRACT(30),
    EXTEND(140);

    @Getter private double demand = 0;
    @Getter private double allowableError = 0;

    State(double output) {
      this(output, 0.01);
    }

    State(double output, double allowable_error) {
      this.demand = output;
      this.allowableError = allowable_error;
    }

    @Override
    public boolean isDisabled() {
      return false;
    }

    @Override
    public ControlState getControlState() {
      return ControlState.POSITION;
    }
  }

  public Request advanceClimbRequest() {
    switch (mState) {
      case ZERO:
        return stateRequest(State.READY);
      case READY:
        return stateRequest(State.RETRACT);
      case RETRACT:
        return stateRequest(State.EXTEND);
      case EXTEND:
        return stateRequest(State.RETRACT);
    }
    return null;
  }

  public Request climbRequest() {
    return new SequentialRequest(
        advanceClimbRequest(),
        // Wait For Climb BB
        advanceClimbRequest(),
        new WaitRequest(1),
        advanceClimbRequest(),
        new WaitRequest(1),
        advanceClimbRequest());
  }

  public void resetClimbStages() {
    setState(State.ZERO);
  }

  @Override
  public void outputTelemetry() {
    RobotVisualizer.updateClimb(getPosition());
    super.outputTelemetry();
  }
}
