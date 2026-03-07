package com.team5817.frc2026.subsystems.Climb;

import com.ctre.phoenix6.hardware.CANcoder;
import com.team5817.frc2026.RobotVisualizer;
import com.team5817.lib.drivers.Actuator.ActuatorIO;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;
import com.team5817.lib.requests.Request;
import com.team5817.lib.requests.SequentialRequest;
import com.team5817.lib.requests.WaitRequest;
import lombok.Getter;

/** Elevator subsystem for controlling the elevator mechanism. */
public class Climb extends StateBasedServoMotorSubsystem<Climb.State> {

  LatchRelease latchRelease;

  public Climb(ServoMotorIO io, ActuatorIO latchIO) {
    super(State.ZERO, io, true);
    this.latchRelease = new LatchRelease(latchIO);
  }

  /** Enum representing the different states of the elevator. */
  public enum State implements ServoState {
    ZERO(0),
    READY(140), // -187 prep
    DOWN(30); // -65 climbed

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
    System.out.println("Advancing climb from " + mState);

    switch (mState) {
      case ZERO:
        latchRelease.setState(LatchRelease.State.RELEASED);
        return stateRequest(State.READY);
      case READY:
        return stateRequest(State.DOWN);
      case DOWN:
        return null;
    }
    return null;
  }

  public Request climbRequest() {
    return new SequentialRequest(
        new Request() {
          @Override
          public void act() {
            advanceClimbRequest().act();
          }
        },
        new WaitRequest(1),
        new Request() {
          @Override
          public void act() {
            advanceClimbRequest().act();
          }
        });
  }

  public void resetClimbStages() {
    setState(State.ZERO);
  }

  @Override
  public void readPeriodicInputs() {
    latchRelease.readPeriodicInputs();
    super.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    latchRelease.writePeriodicOutputs();
    super.writePeriodicOutputs();
  }

  @Override
  public void outputTelemetry() {
    RobotVisualizer.updateClimb(getPosition());
    latchRelease.outputTelemetry();
    super.outputTelemetry();
  }
}
