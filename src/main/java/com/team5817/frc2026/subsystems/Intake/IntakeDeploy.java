package com.team5817.frc2026.subsystems.Intake;

import com.team5817.lib.drivers.Servos.ServoConstants;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;

import edu.wpi.first.math.util.Units;
import lombok.Getter;

/** The IntakeDeploy class controls the deployment mechanism of the intake system. */
public class IntakeDeploy extends StateBasedServoMotorSubsystem<IntakeDeploy.State> {

  /**
   * Constructs a new IntakeDeploy subsystem.
   *
   * @param constants The constants for the servo motor subsystem.
   * @param encoder_constants The constants for the absolute encoder.
   */
  public IntakeDeploy(final ServoConstants constants, ServoMotorIO io) {
  super(IntakeDeploy.State.STOW, io, false);  }

  private double agitateStartTime = 0;
  private static final double agitateAmplitude = Units.inchesToMeters(2);
  private static final double agitateSpeed = 6.0; // radians/sec


  /** Represents the different states of the intake deployment. */
  public enum State implements ServoState {
    STOW(0.025),
    OUT(0.2667),
    AGITATE(0.2667),
    ZERO(0);

    @Getter private double demand = 0;
    @Getter private double allowableError = 0;
    @Getter private boolean disabled = false;

    /**
     * p Constructs a new State.
     *
     * @param output The output value for the state.
     * @param allowable_error The allowable error for the state.
     */
    State(double output) {
      this.demand = output;
      this.allowableError = .1;
    }

    State() {
      this.disabled = true;
    }

    @Override
    public ControlState getControlState() {
      return ControlState.POSITION;
    }
  }

@Override
public void writePeriodicOutputs() {

  if (getState() == State.AGITATE) {

    double time = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    if (agitateStartTime == 0) {
      agitateStartTime = time;
    }

    double elapsed = time - agitateStartTime;

    double center = State.AGITATE.getDemand();

    double dynamicDemand =
        center + agitateAmplitude * Math.sin(elapsed * agitateSpeed);

    super.setPositionSetpoint(dynamicDemand);

  } else {

    agitateStartTime = 0;

    switch (getState().getControlState()) {
      case POSITION:
        super.setPositionSetpoint(getState().getDemand());
        break;

      case VOLTAGE:
        super.applyVoltage(getState().getDemand());
        break;
    }
  }

  // ONLY call motor IO write, not state logic
  super.writePeriodicOutputs();
}

  /** Outputs telemetry data for the subsystem. */
  @Override
  public void outputTelemetry() {
    super.outputTelemetry();
  }
}
