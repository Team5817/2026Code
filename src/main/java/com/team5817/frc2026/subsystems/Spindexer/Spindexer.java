package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.lib.drivers.Rollers.IRollerState;
import com.team5817.lib.drivers.Rollers.RollerSubsystem;
import com.team5817.lib.drivers.Rollers.RollerSubsystem.RollerControlMode;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.Subsystem;

public class Spindexer extends Subsystem {

  public final RollerSubsystem<SpinnerState> spindexer;

  public Spindexer(RollerSubsystemIO spinnerIO, String name) {
    this.spindexer =
        new RollerSubsystem<>(SpinnerState.IDLE, "Indexer/Spindexer " + name, spinnerIO);
  }

  public void setState(SpinnerState spinnerState) {
    spindexer.setState(spinnerState);
  }

  public enum SpinnerState implements IRollerState {
    IDLE(0),
    COUNTERCLOCK(-12),
    CLOCK(12);

    private final double demand;

    SpinnerState(double demand) {
      this.demand = demand;
    }

    @Override
    public double getDemand() {
      return demand;
    }

    @Override
    public RollerControlMode getControlMode() {
      return RollerControlMode.VOLTAGE;
    }

    @Override
    public double getToleranceRadsPerSec() {
      return 0.0;
    }
  }

  @Override
  public void readPeriodicInputs() {
    spindexer.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    spindexer.writePeriodicOutputs();
  }

  @Override
  public boolean checkSystem() {
    return spindexer.allOK();
  }

  @Override
  public void stop() {
    setState(SpinnerState.IDLE);
  }

  @Override
  public void outputTelemetry() {
    spindexer.outputTelemetry();
  }
}
