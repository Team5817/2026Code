package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.frc2026.RobotVisualizer;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.LambdaRequest;
import com.team5817.lib.requests.Request;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class SpindexerGroup extends Subsystem {

  private final SpindexerRoller leftRoller;
  private final SpindexerRoller rightRoller;

  @Getter
  @Setter
  @Accessors(prefix = "m")
  private State mState = State.IDLE;

  public SpindexerGroup(SpindexerRoller leftRoller, SpindexerRoller rightRoller) {
    this.leftRoller = leftRoller;
    this.rightRoller = rightRoller;
  }

  public enum State {
    IDLE(SpindexerRoller.State.IDLE, SpindexerRoller.State.IDLE),
    FEED_TURRET(SpindexerRoller.State.COUNTERCLOCKWISE, SpindexerRoller.State.COUNTERCLOCKWISE),
    FEED_SHOOTER(SpindexerRoller.State.CLOCKWISE, SpindexerRoller.State.CLOCKWISE),
    EXHAUST(SpindexerRoller.State.CLOCKWISE, SpindexerRoller.State.COUNTERCLOCKWISE);

    public final SpindexerRoller.State leftState;
    public final SpindexerRoller.State rightState;

    State(SpindexerRoller.State left, SpindexerRoller.State right) {
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
    leftRoller.spindexer.setState(mState.leftState);
    rightRoller.spindexer.setState(mState.rightState);
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
