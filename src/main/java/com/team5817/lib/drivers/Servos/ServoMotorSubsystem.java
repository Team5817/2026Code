package com.team5817.lib.drivers.Servos;

import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.motion.MotionState;
import com.team254.lib.util.DelayedBoolean;
import com.team254.lib.util.Util;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.Request;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.Logger;

/** Abstract base class for a subsystem with a single sensored servo-mechanism. */
public abstract class ServoMotorSubsystem extends Subsystem {
  
  /* ===================== Constants ===================== */
  
  public static class TalonFXConstants {
    public CanDeviceId id = new CanDeviceId(-1);
    public boolean counterClockwisePositive = true;
    public boolean invert_sensor_phase = false;
  }
  
  protected ServoConstants mConstants;
  protected ServoMotorIO io;
  double ffVolts = 0.0;

  /* ===================== State ===================== */

  @Setter
  @Accessors(prefix = "m")
  protected boolean mHoming = false;

  protected DelayedBoolean mHomingDebounce;

  protected double demand = 0.0;
  protected double velFF = 0.0;
  protected MotionState mMotionStateSetpoint = null;

  protected ServoMotorIOInputsAutoLogged mServoInputs =
      new ServoMotorIOInputsAutoLogged();

  @Getter
  @Accessors(prefix = "m")
  protected ControlState mControlState = ControlState.VOLTAGE;

  /* ===================== Constructor ===================== */

  protected ServoMotorSubsystem(ServoMotorIO io) {
    this.io = io;
    mConstants = io.getConstants();
    mHomingDebounce =
        new DelayedBoolean(Timer.getFPGATimestamp(), mConstants.kHomingTimeout);
    forceZero();
  }

  /* ===================== Control ===================== */

  public enum ControlState {
    POSITION,
    VOLTAGE
  }

  @Override
  public void readPeriodicInputs() {
    io.updateInputs(mServoInputs);
    Logger.processInputs(mConstants.kName, mServoInputs);
  }

  @Override
  public void writePeriodicOutputs() {
    if (mHoming) {
      handleHoming();
      return;
    }


    if (mControlState == ControlState.POSITION) {

      ffVolts = mConstants.kKv * velFF;
    }

    io.setControl(mControlState, demand, ffVolts);
  }
  /* ===================== Homing ===================== */

  public void handleHoming() {
    applyVoltage(mConstants.kHomingOutput * 12.0);

    if (mHomingDebounce.update(
        Timer.getFPGATimestamp(),
        Math.abs(getVelocity()) < mConstants.kHomingVelocityWindow)) {

      forceZero();
      mHomingDebounce =
          new DelayedBoolean(Timer.getFPGATimestamp(), mConstants.kHomingTimeout);
      setPositionSetpoint(mConstants.kHomePosition);
      mHoming = false;
    }
  }

  public void home() {
    setHoming(true);
  }

  /* ===================== Accessors ===================== */

  public double getPositionRotations() {
    return mServoInputs.position_rots;
  }

  public double getPosition() {
    return mServoInputs.position_units;
  }

  public double getVelocity() {
    return mConstants.rotationsToUnits(mServoInputs.velocity_rps);
  }

  public double getPureVelocity() {
    return mServoInputs.velocity_rps;
  }

  public double getVelError() {
    if (mMotionStateSetpoint == null) return 0.0;
    return mConstants.rotationsToUnits(
        mMotionStateSetpoint.vel() - mServoInputs.velocity_rps);
  }

  public boolean hasFinishedTrajectory() {
    return Util.epsilonEquals(
        mServoInputs.active_trajectory_position,
        getSetpoint(),
        Math.max(1, mConstants.kDeadband));
  }

  public double getSetpoint() {
    return mControlState == ControlState.POSITION
        ? mConstants.rotationsToHomedUnits(demand)
        : Double.NaN;
  }

  public double getSetpointHomed() {
    return getSetpoint();
  }

  /* ===================== Commands ===================== */

  public void setPositionSetpoint(double units) {
    demand = constrainRotations(
        mConstants.homeAwareUnitsToRotations(units));
    mControlState = ControlState.POSITION;
  }

  protected double constrainRotations(double rotations) {
    return Util.limit(
        rotations,
        mConstants.mReverseSoftLimitRotations,
        mConstants.mForwardSoftLimitRotations);
  }

  public void applyVoltage(double voltage) {
    mControlState = ControlState.VOLTAGE;
    demand = voltage;
  }

  /* ===================== Utilities ===================== */

  public double getActiveTrajectoryPosition() {
    return mConstants.rotationsToHomedUnits(
        mServoInputs.active_trajectory_position);
  }

  public double getPredictedPositionUnits(double lookaheadSecs) {
    double predicted =
        mServoInputs.active_trajectory_position
            + lookaheadSecs * mServoInputs.active_trajectory_velocity
            + 0.5
                * mServoInputs.active_trajectory_acceleration
                * lookaheadSecs
                * lookaheadSecs;

    if (demand >= mServoInputs.active_trajectory_position) {
      return Math.min(predicted, demand);
    }
    return Math.max(predicted, demand);
  }

  public Request waitToBeOverRequest(double position) {
    return new Request() {
      @Override
      public void act() {}

      @Override
      public boolean isFinished() {
        return mServoInputs.position_units >= position;
      }
    };
  }

  /* ===================== Housekeeping ===================== */

  @Override
  public void zeroSensors() {
    io.zeroSensors();
  }

  public void forceZero() {
    io.forceZeroSensors();
  }

  @Override
  public void outputTelemetry() {
    Logger.recordOutput(mConstants.kName + "/ControlMode", mControlState);
    Logger.recordOutput(mConstants.kName + "/DemandUnits",
        mConstants.rotationsToUnits(demand));
    Logger.recordOutput(mConstants.kName + "/Homing", mHoming);
    Logger.recordOutput(mConstants.kName + "/Feed Forward Volts", ffVolts);
  }

  @Override
  public void rewriteDeviceConfiguration() {
    io.writeConfigs();
  }

  @Override
  public boolean checkDeviceConfiguration() {
    return io.checkDeviceConfiguration();
  }
}
