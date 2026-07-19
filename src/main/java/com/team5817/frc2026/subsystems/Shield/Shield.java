package com.team5817.frc2026.subsystems.Shield;

import com.team5817.frc2026.RobotVisualizer;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;
import com.team5817.lib.requests.Request;
import lombok.Getter;

public class Shield extends StateBasedServoMotorSubsystem<Shield.State> {

  public Shield(ServoMotorIO io) {
    super(State.ZERO, io, true);
  }

  public enum State implements ServoState {
    ZERO(0.0, 0.02),
    EXTENDED(0.4, 0.02),
    RETRACTED(0.0, 0.01);

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

  public Request advanceShieldRequest() {
    if (mDesiredState == State.EXTENDED) {
      return stateRequest(State.RETRACTED);
    } else {
      return stateRequest(State.EXTENDED);
    }
  }

  public Request zeroRequest() {
    return stateRequest(State.ZERO);
  }

  @Override
  public void outputTelemetry() {
    RobotVisualizer.updateShield(getPosition());
    super.outputTelemetry();
  }
}
