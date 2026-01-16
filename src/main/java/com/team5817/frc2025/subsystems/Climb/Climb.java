package com.team5817.frc2025.subsystems.Climb;

import com.team5817.frc2025.RobotVisualizer;
import com.team5817.lib.Util;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;

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
  final static double kStrictError = 0;
  final static double kMediumError = 0;
  final static double kLenientError = 0;

  /**
   * Enum representing the different states of the elevator.
   */
  public enum State implements ServoState {
    ZERO(0, kMediumError),
    READY(0, kMediumError),
    L3(0, kStrictError),
    L2(0, kStrictError), 
    L1(0, kStrictError);


    @Getter
    private double demand = 0;
    @Getter
    private double allowableError = 0;

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


   @Override
  public void outputTelemetry() {
    //RobotVisualizer.updateElevatorHeight(getPosition());

   // Logger.recordOutput("Elevator/Offset", );

    super.outputTelemetry();
  }
} 
