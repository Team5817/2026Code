package com.team5817.frc2025.subsystems.Climb;

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
private double scoringOffset = 0;
    private double distanceFromRung = 0;
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
    private InterpolatingDoubleTreeMap map;

    State(double output, double allowable_error) {
      this(output, allowable_error, null);
    }

    State(double output, double allowable_error, InterpolatingDoubleTreeMap map) {
      this.demand = output;
      this.allowableError = allowable_error;
      this.map = map;
    }

    public double getTrackedOutput(double distanceFromRung) {
      if (map == null) {
        return demand;
      }
      double des = this.demand + map.get(distanceFromRung);
      des = Util.limit(des, ClimbConstants.kClimbServoConstants.kMinUnitsLimit,
          ClimbConstants.kClimbServoConstants.kMaxUnitsLimit);
      return des;
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

    public void updateRungDistance(double dist) {
        this.distanceFromRung = dist;
    }

  @Override
  public void writePeriodicOutputs() {
    double trackedOutput = 0; //mState.getTrackedOutput(distanceFromRung);
    if (mState == State.L1 || mState == State.L2 || mState == State.L3)
     // trackedOutput += scoringOffset;
        trackedOutput = mState.demand;
    
    setPositionSetpoint(trackedOutput);

    super.writePeriodicOutputs();
  }



}

 /*  @Override
  public void outputTelemetry() {
    RobotVisualizer.updateElevatorHeight(getPosition());

    Logger.recordOutput("Elevator/Offset", this.scoringOffset);

    super.outputTelemetry();
  }
} */
