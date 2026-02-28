package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.lib.drivers.Rollers.IRollerState;
import com.team5817.lib.drivers.Rollers.RollerSubsystem;
import com.team5817.lib.drivers.Rollers.RollerSubsystem.RollerControlMode;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.Subsystem;

public class Spindexer extends Subsystem {

  public final RollerSubsystem<SpinnerState> spindexer;
  public final RollerSubsystem<TunnelState> tunnel;

  public Spindexer(RollerSubsystemIO spinnerIO, RollerSubsystemIO tunnelIO, String name) {
    this.spindexer =
        new RollerSubsystem<SpinnerState>(SpinnerState.IDLE, "Spindexer" + name, spinnerIO);
    this.tunnel = new RollerSubsystem<TunnelState>(TunnelState.IDLE, "Tunnel " + name, tunnelIO);
  }

  public void setState(SpinnerState spinnerState) {
    spindexer.setState(spinnerState);
    switch (spinnerState) {
      case IDLE:
        tunnel.setState(TunnelState.IDLE);
        break;
      case COUNTERCLOCK:
        tunnel.setState(TunnelState.IN);
        break;
      case CLOCK:
        tunnel.setState(TunnelState.AWAY);
        break;
      case EXHAUST:
        tunnel.setState(TunnelState.EXHAUST);
        break;
    }
  }

  public enum SpinnerState implements IRollerState {
    IDLE(0),
    COUNTERCLOCK(10),
    CLOCK(-10),
    EXHAUST(-12);

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

  public enum TunnelState implements IRollerState {
    IDLE(0),
    IN(-10),
    AWAY(10),
    EXHAUST(12);

    private final double demand;

    TunnelState(double demand) {
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
    tunnel.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    spindexer.writePeriodicOutputs();
    tunnel.writePeriodicOutputs();
  }

  @Override
  public boolean checkSystem() {
    return spindexer.allOK() && tunnel.allOK();
  }

  @Override
  public void outputTelemetry() {
    spindexer.outputTelemetry();
    tunnel.outputTelemetry();
  }
}
