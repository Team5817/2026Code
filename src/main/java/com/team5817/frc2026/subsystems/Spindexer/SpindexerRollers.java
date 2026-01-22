package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.frc2026.subsystems.Spindexer.SpindexerConstants.SpindexerState;
import com.team5817.lib.drivers.Rollers.RollerSubsystem;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.Logger;

public class SpindexerRollers extends Subsystem {

  private final RollerSubsystem<SpindexerState> Spindexer;

  @Setter
  @Getter
  @Accessors(prefix = "m")
  private State mState = State.IDLE;

  public SpindexerRollers(RollerSubsystemIO SpindexerIO) {
    this.Spindexer =
        new RollerSubsystem<SpindexerState>(SpindexerState.IDLE, "Intake/Spindexer", SpindexerIO);
  }

  public enum State {
    IDLE(SpindexerState.IDLE),
    CLOCKWISE(SpindexerState.CLOCKWISE),
    COUNTERCLOCKWISE(SpindexerState.COUNTERCLOCKWISE);

    
   
    @Getter
    private final SpindexerState spindexerState;
    

    State(SpindexerState SpindexerState) {
      this.spindexerState = SpindexerState;
    }
  }

  @Override
  public void readPeriodicInputs() {
    Spindexer.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    Spindexer.writePeriodicOutputs();
  }

  @Override
  public boolean checkSystem() {
    return Spindexer.allOK();
  }

  public Request stateRequest(State state) {
    setState(state);
    return new ParallelRequest(Spindexer.stateRequest(state.spindexerState));
  }

  @Override
  public void outputTelemetry() {
    Logger.recordOutput("Spindexer/RollerState", getState());
  }
}
