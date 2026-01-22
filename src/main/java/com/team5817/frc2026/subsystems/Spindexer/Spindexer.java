package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.Request;
import com.team5817.lib.requests.ParallelRequest;

public class Spindexer extends Subsystem {

    private final SpindexerRollers rollers;

    private State mState = State.IDLE;

    public Spindexer(SpindexerRollers rollers) {
        this.rollers = rollers;
    }

    public enum State {
        IDLE(SpindexerConstants.SpindexerState.IDLE, SpindexerConstants.SpindexerState.IDLE),
        FEED_TURRET(SpindexerConstants.SpindexerState.COUNTERCLOCKWISE, SpindexerConstants.SpindexerState.COUNTERCLOCKWISE),
        FEED_SHOOTER(SpindexerConstants.SpindexerState.CLOCKWISE, SpindexerConstants.SpindexerState.CLOCKWISE),
        EXHAUST(SpindexerConstants.SpindexerState.CLOCKWISE, SpindexerConstants.SpindexerState.COUNTERCLOCKWISE);

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
            rollers.stateRequest(mapToRollerState(state.leftState, state.rightState))
        );
    }

    private SpindexerRollers.State mapToRollerState(
        SpindexerConstants.SpindexerState left, 
        SpindexerConstants.SpindexerState right
    ) 
    
    {
        if (left == right) {
            switch (left) {
                case IDLE: return SpindexerRollers.State.IDLE;
                case CLOCKWISE: return SpindexerRollers.State.CLOCKWISE;
                case COUNTERCLOCKWISE: return SpindexerRollers.State.COUNTERCLOCKWISE;
            }
        }
        if (left == SpindexerConstants.SpindexerState.CLOCKWISE || right == SpindexerConstants.SpindexerState.CLOCKWISE) {
            return SpindexerRollers.State.CLOCKWISE;
        } else if (left == SpindexerConstants.SpindexerState.COUNTERCLOCKWISE || right == SpindexerConstants.SpindexerState.COUNTERCLOCKWISE) {
            return SpindexerRollers.State.COUNTERCLOCKWISE;
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
