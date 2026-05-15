package com.team5817.frc2026.subsystems.Indexer;

import com.team5817.frc2026.RobotVisualizer;
import com.team5817.lib.drivers.Rollers.IRollerState;
import com.team5817.lib.drivers.Rollers.RollerSubsystem;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.LambdaRequest;
import com.team5817.lib.requests.Request;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class Indexer extends Subsystem {

  private final RollerSubsystem<SpindexerState> spindexer;
  private final RollerSubsystem<TunnelState> tunnel;

  @Getter
  @Setter
  @Accessors(prefix = "m")
  private State mState = State.IDLE;

  public Indexer(RollerSubsystemIO spindexerIO, RollerSubsystemIO tunnelIO) {
    spindexer = new RollerSubsystem<>(SpindexerState.IDLE, "Indexer/Spindexer", spindexerIO);
    tunnel = new RollerSubsystem<>(TunnelState.IDLE, "Indexer/Tunnel", tunnelIO);
  }

  public enum State {
    IDLE(SpindexerState.IDLE, TunnelState.IDLE),
    FEED(SpindexerState.COUNTERCLOCK, TunnelState.IN),
    SPINUP(SpindexerState.IDLE, TunnelState.IN),
    EXHAUST(SpindexerState.CLOCK, TunnelState.EXHAUST);

    public final SpindexerState spindexerState;
    public final TunnelState tunnelState;

    State(SpindexerState spindexerState, TunnelState tunnelState) {
      this.spindexerState = spindexerState;
      this.tunnelState = tunnelState;
    }
  }

  public Request stateRequest(State state) {
    return new LambdaRequest(() -> setState(state));
  }

  public enum SpindexerState implements IRollerState {
    IDLE(0),
    COUNTERCLOCK(-9),
    CLOCK(10);

    private final double demand;

    SpindexerState(double demand) {
      this.demand = demand;
    }

    @Override
    public double getDemand() {
      return demand;
    }

    @Override
    public RollerSubsystem.RollerControlMode getControlMode() {
      return RollerSubsystem.RollerControlMode.VOLTAGE;
    }

    @Override
    public double getToleranceRadsPerSec() {
      return 0.0;
    }
  }

  public enum TunnelState implements IRollerState {
    IDLE(0),
    IN(10),
    EXHAUST(-7);

    private final double demand;

    TunnelState(double demand) {
      this.demand = demand;
    }

    @Override
    public double getDemand() {
      return demand;
    }

    @Override
    public RollerSubsystem.RollerControlMode getControlMode() {
      return RollerSubsystem.RollerControlMode.VOLTAGE;
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
    spindexer.setDesiredState(mState.spindexerState);
    tunnel.setDesiredState(mState.tunnelState);

    spindexer.writePeriodicOutputs();
    tunnel.writePeriodicOutputs();
  }

  @Override
  public boolean checkSystem() {
    return spindexer.allOK() && tunnel.allOK();
  }

  @Override
  public void stop() {
    setState(State.IDLE);
  }

  @Override
  public void outputTelemetry() {
    RobotVisualizer.updateSpindexerLeft(spindexer.getVelocity());

    spindexer.outputTelemetry();
    tunnel.outputTelemetry();
    super.outputTelemetry();
  }
}
