package com.team5817.lib.drivers.Rollers;

import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.Request;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem<S extends Enum<S> & IRollerState> extends Subsystem {
  private final String inputsName;
  private final RollerSubsystemIO io;
  protected final RollerSubsystemIOInputsAutoLogged inputs =
      new RollerSubsystemIOInputsAutoLogged();
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;
  private final Alert tempFault;

  private boolean brakeModeEnabled = true;

  @Getter
  @Setter
  @Accessors(prefix = "m")
  private S mDesiredState;

  protected boolean atState = true;

  public boolean atState() {
    return atState;
  }

  public enum RollerControlMode {
    VOLTAGE,
    TORQUE,
    VELOCITY
  }

  public RollerSubsystem(S initialState, String name, String inputsName, RollerSubsystemIO io) {
    this.mDesiredState = initialState;
    this.inputsName = name;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    tempFault = new Alert(name + " motor too hot! 🥵", Alert.AlertType.kWarning);
  }

  public void readPeriodicInputs() {
    io.updateInputs(inputs);
    Logger.processInputs(inputsName, inputs);

    if (mDesiredState.getControlMode() == RollerControlMode.VELOCITY) {
      double error = mDesiredState.getDemand() - inputs.data.velocityRotsPerSec();
      atState = Math.abs(error) < mDesiredState.getToleranceRadsPerSec();
    } else {
      atState = true;
    }

    disconnected.set(!motorConnectedDebouncer.calculate(inputs.data.connected()));
    tempFault.set(inputs.data.tempFault());
  }

  public void writePeriodicOutputs() {
    switch (mDesiredState.getControlMode()) {
      case TORQUE:
        io.runTorqueCurrent(mDesiredState.getDemand());
        break;
      case VELOCITY:
        io.runVelocity(mDesiredState.getDemand());
        break;
      case VOLTAGE:
        io.runVolts(mDesiredState.getDemand());
        break;
    }

    double error = mDesiredState.getDemand() - inputs.data.velocityRotsPerSec();

    Logger.recordOutput(inputsName + "/State", mDesiredState);
    Logger.recordOutput(inputsName + "/Control Mode", mDesiredState.getControlMode());
    Logger.recordOutput(inputsName + "/Desired", mDesiredState.getDemand());
    Logger.recordOutput(inputsName + "/Position", Math.toDegrees(inputs.data.positionRads()));
    Logger.recordOutput(inputsName + "/Velocity", inputs.data.velocityRotsPerSec());
    Logger.recordOutput(inputsName + "/Error", error);
    Logger.recordOutput(inputsName + "/BrakeModeEnabled", brakeModeEnabled);
    Logger.recordOutput(inputsName + "/atState", atState);
  }

  public RollerSubsystem(S initialState, String name, RollerSubsystemIO io) {
    this(initialState, name, String.join(name, "Inputs"), io);
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(enabled);
  }

  public double getTorqueCurrent() {
    return inputs.data.torqueCurrentAmps();
  }

  public double getVelocity() {
    return inputs.data.velocityRotsPerSec();
  }

  public Request stateRequest(S newState) {
    return new Request() {
      @Override
      public void act() {
        setDesiredState(newState);
      }
    };
  }

  public boolean allOK() {
    return !disconnected.get() && !tempFault.get();
  }
}
