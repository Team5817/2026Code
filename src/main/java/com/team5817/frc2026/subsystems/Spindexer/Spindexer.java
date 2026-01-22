package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;

public class Spindexer extends Subsystem {

  private final SpindexerRollers rollers;

  private State mState = State.IDLE;

  public Spindexer(SpindexerRollers rollers) {
    this.rollers = rollers;
  }

  public enum State {
    IDLE(SpindexerConstants.SpindexerState.IDLE, SpindexerConstants.SpindexerState.IDLE),
    FEED_TURRET(SpindexerConstants.SpindexerState.INTAKING, SpindexerConstants.SpindexerState.IDLE),
    FEED_SHOOTER(
        SpindexerConstants.SpindexerState.IDLE, SpindexerConstants.SpindexerState.INTAKING),
    REVERSE_ALL(
        SpindexerConstants.SpindexerState.EXHAUST, SpindexerConstants.SpindexerState.EXHAUST);

    public final SpindexerConstants.SpindexerState leftState;
    public final SpindexerConstants.SpindexerState rightState;

    State(SpindexerConstants.SpindexerState left, SpindexerConstants.SpindexerState right) {
      this.leftState = left;
      this.rightState = right;
    }
  }

  public void setState(State state) {
    mState = state;
    stateRequest(state).act();
  }

  public State getState() {
    return mState;
  }

  public Request stateRequest(State state) {
    return new ParallelRequest(
        rollers.stateRequest(mapToRollerState(state.leftState, state.rightState)));
  }

  private SpindexerRollers.State mapToRollerState(
      SpindexerConstants.SpindexerState left, SpindexerConstants.SpindexerState right) {
    // Simple mapping: if both same, return that, else default to left INTAKING priority
    if (left == right) {
      switch (left) {
        case IDLE:
          return SpindexerRollers.State.IDLE;
        case INTAKING:
          return SpindexerRollers.State.INTAKING;
        case EXHAUST:
          return SpindexerRollers.State.EXHAUST;
      }
    }
    // Mixed states
    if (left == SpindexerConstants.SpindexerState.INTAKING
        || right == SpindexerConstants.SpindexerState.INTAKING) {
      return SpindexerRollers.State.INTAKING;
    } else if (left == SpindexerConstants.SpindexerState.EXHAUST
        || right == SpindexerConstants.SpindexerState.EXHAUST) {
      return SpindexerRollers.State.EXHAUST;
    }
    return SpindexerRollers.State.IDLE;
  }

  @Override
  public void readPeriodicInputs() {
    rollers.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    rollers.writePeriodicOutputs();
  }

  @Override
  public boolean checkSystem() {
    return rollers.checkSystem();
  }

  @Override
  public void stop() {
    setState(State.IDLE);
  }
}
