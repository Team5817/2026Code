package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.frc2026.RobotVisualizer;
import com.team5817.frc2026.subsystems.Spindexer.Tunnel.TunnelState;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.LambdaRequest;
import com.team5817.lib.requests.Request;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class Indexer extends Subsystem {

  private final Spindexer leftRoller;
  private final Spindexer rightRoller;
  private final Tunnel tunnel;

  @Getter
  @Setter
  @Accessors(prefix = "m")
  private State mState = State.IDLE;

  public Indexer(Spindexer leftRoller, Spindexer rightRoller, Tunnel tunnel) {
    this.leftRoller = leftRoller;
    this.rightRoller = rightRoller;
    this.tunnel = tunnel;
  }

  public enum State {
    IDLE(Spindexer.SpinnerState.IDLE, Spindexer.SpinnerState.IDLE, Tunnel.TunnelState.IDLE),

    FEED(
        Spindexer.SpinnerState.COUNTERCLOCK,
        Spindexer.SpinnerState.COUNTERCLOCK,
        Tunnel.TunnelState.IN),
    SPINUP(
      Spindexer.SpinnerState.IDLE,
      Spindexer.SpinnerState.IDLE,
      TunnelState.IN),
    EXHAUST(
        Spindexer.SpinnerState.COUNTERCLOCK,
        Spindexer.SpinnerState.CLOCK,
        Tunnel.TunnelState.EXHAUST);

    public final Spindexer.SpinnerState leftState;
    public final Spindexer.SpinnerState rightState;
    public final Tunnel.TunnelState tunnelState;

    State(Spindexer.SpinnerState left, Spindexer.SpinnerState right, Tunnel.TunnelState tunnel) {
      this.leftState = left;
      this.rightState = right;
      this.tunnelState = tunnel;
    }
  }

  public Request stateRequest(State state) {
    return new LambdaRequest(() -> setState(state));
  }

  @Override
  public void readPeriodicInputs() {
    leftRoller.readPeriodicInputs();
    rightRoller.readPeriodicInputs();
    tunnel.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    leftRoller.setState(mState.leftState);
    rightRoller.setState(mState.rightState);
    tunnel.setState(mState.tunnelState);

    leftRoller.writePeriodicOutputs();
    rightRoller.writePeriodicOutputs();
    tunnel.writePeriodicOutputs();
  }

  @Override
  public boolean checkSystem() {
    return leftRoller.checkSystem() && rightRoller.checkSystem() && tunnel.checkSystem();
  }

  @Override
  public void stop() {
    setState(State.IDLE);
  }

  @Override
  public void outputTelemetry() {
    RobotVisualizer.updateSpindexerLeft(leftRoller.spindexer.getVelocity());
    RobotVisualizer.updateSpindexerRight(rightRoller.spindexer.getVelocity());

    leftRoller.outputTelemetry();
    rightRoller.outputTelemetry();
    tunnel.outputTelemetry();

    super.outputTelemetry();
  }
}
