package com.team5817.frc2026.subsystems.Intake;

import com.team5817.lib.drivers.Rollers.IRollerState;
import com.team5817.lib.drivers.Rollers.RollerSubsystem;
import com.team5817.lib.drivers.Rollers.RollerSubsystem.RollerControlMode;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends Subsystem {

  private final RollerSubsystem<State> feeder;

  @Setter
  @Getter
  @Accessors(prefix = "m")
  private State mState = State.IDLE;

  private static DoubleSupplier voltageSupplier = () -> -10.0;

  public IntakeRollers(RollerSubsystemIO FeederIO, DoubleSupplier VoltageSupplier) {
    this.feeder = new RollerSubsystem<State>(State.IDLE, "Intake/Feeder", FeederIO);
    voltageSupplier = VoltageSupplier;
  }

  public enum State implements IRollerState {
    IDLE(0),
    INTAKING(-7),
    EXHAUST(8);

    DoubleSupplier demand;
    @Getter RollerControlMode controlMode = RollerControlMode.VOLTAGE;

    State(double voltage) {
      this.demand = () -> voltage;
    }

    State() {
      this.demand = voltageSupplier::getAsDouble;
    }

    @Override
    public double getDemand() {
      return demand.getAsDouble();
    }
  }

  @Override
  public void readPeriodicInputs() {
    feeder.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    feeder.writePeriodicOutputs();
  }

  @Override
  public boolean checkSystem() {
    return feeder.allOK();
  }

  public Request stateRequest(State state) {
    setState(state);
    return new ParallelRequest(feeder.stateRequest(state));
  }

  @Override
  public void outputTelemetry() {
    Logger.recordOutput("Intake/RollerState", getState());
  }
}
