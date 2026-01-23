package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.lib.drivers.Rollers.IRollerState;
import com.team5817.lib.drivers.Rollers.RollerSubsystem;
import com.team5817.lib.drivers.Rollers.RollerSubsystem.RollerControlMode;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.Subsystem;

public class SpindexerRoller extends Subsystem {

  public final RollerSubsystem<State> spindexer;

  private String name;

  public SpindexerRoller(RollerSubsystemIO RollerIO, String name) {
    this.name = name;
    this.spindexer = new RollerSubsystem<State>(State.IDLE, "Spindexer" + name, RollerIO);
  }

  public enum State implements IRollerState {
    IDLE(0),
    CLOCKWISE(10),
    COUNTERCLOCKWISE(-10),
    EXHAUST(-12);

    private final double demand;

    State(double demand) {
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
  public void outputTelemetry() {
    super.outputTelemetry();
  }
}
