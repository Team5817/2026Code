package com.team5817.frc2026.subsystems.Elevator;

import com.team5817.frc2026.RobotVisualizer;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;
import com.team5817.lib.requests.Request;
import com.team5817.lib.requests.SequentialRequest;
import com.team5817.lib.requests.WaitRequest;
import lombok.Getter;

public class Elevator extends StateBasedServoMotorSubsystem<Elevator.State> {

  public Elevator(ServoMotorIO io) {
    super(State.ZERO, io, false);
  }

  public enum State implements ServoState {
    ZERO(0.0, 0.0),
    EXTENDED(0.4, 0.02),
    RETRACTED(0.1, 0.01);

    @Getter private double demand = 0;
    @Getter private double allowableError = 0;

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
    System.out.println("Advancing climb from " + mDesiredState);

    switch (mDesiredState) {
      case ZERO:
        return stateRequest(State.EXTENDED);
      case EXTENDED:
        return stateRequest(State.RETRACTED);
      case RETRACTED:
        return stateRequest(State.EXTENDED);
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
    setDesiredState(State.ZERO);
  }

  @Override
  public void outputTelemetry() {
    RobotVisualizer.updateClimb(getPosition());
    super.outputTelemetry();
  }
}
