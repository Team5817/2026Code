package com.team5817.frc2026.subsystems.Intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.lib.util.Util;
import com.team5817.lib.drivers.Servos.ServoConstants;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

/** The IntakeDeploy class controls the deployment mechanism of the intake system. */
public class IntakeDeploy extends StateBasedServoMotorSubsystem<IntakeDeploy.State> {

  public IntakeDeploy(final ServoConstants constants, ServoMotorIO io) {
    super(IntakeDeploy.State.OUT, io, true);
  }

  /** Represents the different states of the intake deployment. */
  public enum State implements ServoState {
    OUT(0.3175),
    DISABLED(),
    SQUEEZE(0.2),
    ZERO(0);

    @Getter private double demand = 0;
    @Getter private double allowableError = 0;
    @Getter private boolean disabled = false;
    @Getter private NeutralModeValue neutralMode = NeutralModeValue.Brake;

    /**
     * p Constructs a new State.
     *
     * @param output The output value for the state.
     * @param allowable_error The allowable error for the state.
     */
    State(double output) {
      this.demand = output;
      this.allowableError = .03;
    }

    State() {
      this.disabled = true;
      this.neutralMode = NeutralModeValue.Coast;
    }

    @Override
    public ControlState getControlState() {
      return ControlState.POSITION;
    }
  }

  @Override
  public void writePeriodicOutputs() {
    boolean out =
        Util.epsilonEquals(getPosition(), State.OUT.getDemand(), State.OUT.getAllowableError());
    Logger.recordOutput("Intake/Rack/Out", out);
    if (out && getDesiredState() == State.OUT) setDesiredState(State.DISABLED);
    super.writePeriodicOutputs();
  }
}
