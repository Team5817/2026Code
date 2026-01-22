package com.team5817.frc2026.subsystems.Intake;

import com.team5817.frc2026.subsystems.Intake.IntakeConstants.RollerConstants.FeederState;
import com.team5817.lib.drivers.Rollers.RollerSubsystem;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.Logger;

public class IntakeRollers extends Subsystem {

  private final RollerSubsystem<FeederState> feeder;

  @Setter
  @Getter
  @Accessors(prefix = "m")
  private State mState = State.IDLE;

  public IntakeRollers(RollerSubsystemIO FeederIO) {
    this.feeder = new RollerSubsystem<FeederState>(FeederState.IDLE, "Intake/Feeder", FeederIO);
  }

  public enum State {
    IDLE(FeederState.IDLE),
    INTAKING(FeederState.INTAKING),
    EXHAUST(FeederState.EXHAUST);

    @Getter private final FeederState feederState;

    State(FeederState feederState) {
      this.feederState = feederState;
    }
  }

  @Override
  public void readPeriodicInputs() {
    feeder.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    feeder.writePeriodicOutputs();
  }

  @Override
  public boolean checkSystem() {
    return feeder.allOK();
  }

  public Request stateRequest(State state) {
    setState(state);
    return new ParallelRequest(feeder.stateRequest(state.feederState));
  }

  @Override
  public void outputTelemetry() {
    Logger.recordOutput("Intake/RollerState", getState());
  }
}
