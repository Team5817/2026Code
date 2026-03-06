package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.lib.drivers.Rollers.IRollerState;
import com.team5817.lib.drivers.Rollers.RollerSubsystem;
import com.team5817.lib.drivers.Rollers.RollerSubsystem.RollerControlMode;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.Subsystem;

public class Tunnel extends Subsystem {

  public final RollerSubsystem<TunnelState> tunnel;

  public Tunnel(RollerSubsystemIO io) {
    this.tunnel = new RollerSubsystem<>(TunnelState.IDLE, "Tunnel", io);
  }

  public void setState(TunnelState state) {
    tunnel.setState(state);
  }

  public enum TunnelState implements IRollerState {
    IDLE(0),
    IN(11),
    EXHAUST(-11);

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
    tunnel.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    tunnel.writePeriodicOutputs();
  }

  @Override
  public boolean checkSystem() {
    return tunnel.allOK();
  }

  @Override
  public void stop() {
    setState(TunnelState.IDLE);
  }

  @Override
  public void outputTelemetry() {
    tunnel.outputTelemetry();
  }
}
