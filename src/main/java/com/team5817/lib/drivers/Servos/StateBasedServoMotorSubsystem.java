package com.team5817.lib.drivers.Servos;

import com.team5817.lib.Util;
import com.team5817.lib.requests.Request;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.Logger;

public class StateBasedServoMotorSubsystem<S extends Enum<S> & ServoState>
    extends ServoMotorSubsystem {
  @Getter
  @Setter
  @Accessors(prefix = "m")
  protected S mDesiredState;

  private final boolean allowAutoStateOutput;
  protected boolean atState = false;

  public boolean atState() {
    return atState;
  }

  public StateBasedServoMotorSubsystem(
      S initialState, ServoMotorIO io, boolean enableAutoStateOutput) {
    super(io);
    this.mDesiredState = initialState;
    this.allowAutoStateOutput = enableAutoStateOutput;
  }

  public StateBasedServoMotorSubsystem(S initialState, ServoMotorIO io) {
    this(initialState, io, true);
  }

  @Override
  public void writePeriodicOutputs() {
    if (allowAutoStateOutput)
      switch (mDesiredState.getControlState()) {
        case POSITION:
          super.setPositionSetpoint(mDesiredState.getDemand());
          break;
        case VOLTAGE:
          super.applyVoltage(mDesiredState.getDemand());
      }

    if (mDesiredState.isDisabled()) super.applyVoltage(0);

    super.writePeriodicOutputs();
  }

  @Override
  public void readPeriodicInputs() {
    super.readPeriodicInputs();
    atState =
        Util.epsilonEquals(
            getPosition() - mConstants.kHomePosition,
            mConstants.rotationsToUnits(demand),
            mDesiredState.getAllowableError());
    if (mDesiredState.isDisabled() || mControlState != ControlState.POSITION) atState = true;
  }

  @Override
  public void outputTelemetry() {
    Logger.recordOutput(mConstants.kName + "/AtState", atState);
    Logger.recordOutput(mConstants.kName + "/State", mDesiredState);
    super.outputTelemetry();
  }

  /**
   * Creates a request to change the state of the intake deployment.
   *
   * @param _wantedState The desired state.
   * @return The request to change the state.
   */
  public Request stateRequest(S _wantedState) {
    return new Request() {
      @Override
      public void act() {
        if (mControlState != ControlState.POSITION) {
          mControlState = ControlState.POSITION;
        }
        setDesiredState(_wantedState);
      }

      @Override
      public boolean isFinished() {
        return atState;
      }
    };
  }
}
