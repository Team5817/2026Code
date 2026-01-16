package com.team5817.frc2025.subsystems.Climb;

import javax.security.auth.PrivateCredentialPermission;

import com.fasterxml.jackson.core.util.DefaultIndenter;
import com.team5817.frc2025.RobotVisualizer;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;
import com.team5817.lib.requests.Request;
import com.team5817.lib.requests.SequentialRequest;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lombok.Getter;


/**
 * Elevator subsystem for controlling the elevator mechanism.
 */
public class Climb extends StateBasedServoMotorSubsystem<Climb.State> {

  /**
   * Constructs an Elevator with the given constants.
   * 
   * @param constants the constants for the elevator
   */
  public Climb(ServoMotorIO io) {
    super(State.ZERO, io, false);
  }


  /**
   * Enum representing the different states of the elevator.
   */
  public enum State implements ServoState {
    ZERO(0),
    READY(0),
    RETRACT(0),
    EXTEND(0);

    @Getter
    private double demand = 0;
    @Getter
    private double allowableError = 0;

    State(double output){
        this(output,0.01);
    }
    State(double output, double allowable_error) {
      this(output, allowable_error, null);
    }

    State(double output, double allowable_error, InterpolatingDoubleTreeMap map) {
      this.demand = output;
      this.allowableError = allowable_error;
    }

    @Override
    public boolean isDisabled() {
      return false;
    }

    @Override
    public ControlState getControlState() {
      return ControlState.POSITION;
    }

  }

   void advanceClimbSequence(){
    switch (mState) {
      case ZERO:
        setState(State.READY);
        break;
      case READY:
        setState(State.RETRACT);
        break;
      case RETRACT:
        setState(State.EXTEND);
        break;
      case  EXTEND:
        setState(State.RETRACT);
        break;
    }
  }
  void resetClimbStages(){
    setState(State.ZERO);
  }

   @Override
  public void outputTelemetry() {


    super.outputTelemetry();
  }
} 
