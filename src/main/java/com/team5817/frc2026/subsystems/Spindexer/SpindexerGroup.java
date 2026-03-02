package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.frc2026.RobotVisualizer;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.LambdaRequest;
import com.team5817.lib.requests.Request;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class SpindexerGroup extends Subsystem {

  private final Spindexer leftRoller;
  private final Spindexer rightRoller;

  @Getter
  @Setter
  @Accessors(prefix = "m")
  private State mState = State.IDLE;

  public SpindexerGroup(Spindexer leftRoller, Spindexer rightRoller) {
    this.leftRoller = leftRoller;
    this.rightRoller = rightRoller;
  }

  public enum State {
    IDLE(Spindexer.SpinnerState.IDLE, Spindexer.SpinnerState.IDLE),
    FEED_TURRET(Spindexer.SpinnerState.COUNTERCLOCK, Spindexer.SpinnerState.COUNTERCLOCK),
    FEED_SHOOTER(Spindexer.SpinnerState.COUNTERCLOCK, Spindexer.SpinnerState.COUNTERCLOCK),
    FEED_BOTH(Spindexer.SpinnerState.CLOCK, Spindexer.SpinnerState.COUNTERCLOCK),
    EXHAUST(Spindexer.SpinnerState.COUNTERCLOCK, Spindexer.SpinnerState.CLOCK);

    public final Spindexer.SpinnerState leftState;
    public final Spindexer.SpinnerState rightState;

    State(Spindexer.SpinnerState left, Spindexer.SpinnerState right) {
      this.leftState = left;
      this.rightState = right;
    }
  }

  public Request stateRequest(State state) {
    return new LambdaRequest(() -> setState(state));
  }

  @Override
  public void readPeriodicInputs() {
    leftRoller.readPeriodicInputs();
    rightRoller.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    leftRoller.setState(mState.leftState);
    rightRoller.setState(mState.rightState);
    leftRoller.writePeriodicOutputs();
    rightRoller.writePeriodicOutputs();
  }

  @Override
  public boolean checkSystem() {
    return leftRoller.checkSystem() && rightRoller.checkSystem();
  }

  @Override
  public void stop() {
    setState(State.IDLE);
  }

  @Override
  public void outputTelemetry() {
    RobotVisualizer.updateSpindexerLeft(leftRoller.spindexer.getVelocity());
    RobotVisualizer.updateSpindexerRight(rightRoller.spindexer.getVelocity());
    rightRoller.outputTelemetry();
    leftRoller.outputTelemetry();
    super.outputTelemetry();
  }
}
