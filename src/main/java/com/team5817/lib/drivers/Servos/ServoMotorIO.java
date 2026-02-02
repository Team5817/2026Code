package com.team5817.lib.drivers.Servos;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.lib.drivers.Servos.ServoMotorSubsystem.ControlState;
import org.littletonrobotics.junction.AutoLog;

public interface ServoMotorIO {

  @AutoLog
  public static class ServoMotorIOInputs {
    public double timestamp;
    public double position_rots = 0; // motor rotations
    public double position_units;
    public double velocity_rps;
    public double velocity_unitspS;
    public double prev_vel_rps;
    public double output_percent;
    public double output_voltage;
    public double main_stator_current;
    public double main_supply_current;
    public double error_rotations;
    public boolean reset_occured;
    public double active_trajectory_position;
    public double active_trajectory_velocity;
    public double active_trajectory_acceleration;
    public double rotor_position;
  }

  public default ServoConstants getConstants() {
    return new ServoConstants();
  }

  public default void updateInputs(ServoMotorIOInputs inputs) {}

  public default void setControl(ServoMotorSubsystem.ControlState mControlState, double demand, double ffVolts) {
    if (mControlState == ControlState.POSITION) {
      runPosition(demand, ffVolts);
    } else if (mControlState == ControlState.VOLTAGE) {
      runVoltage(demand);
    }
  }
  public default void setControl(ServoMotorSubsystem.ControlState mControlState, double demand){
    setControl(mControlState, demand,0);
  }

  public default void runPosition(double units, double ffVolts) {}

  public default void runVoltage(double volts) {}

  public default void zeroSensors() {
    zeroSensors(0);
  }

  public default void zeroSensors(double newPose) {}

  public default void forceZeroSensors() {}

  public default void setNeutralMode(NeutralModeValue mode) {}

  public default void setStatorCurrentLimit(double limit, boolean enable) {}

  public default void writeConfigs() {}

  public default boolean checkDeviceConfiguration() {
    return true;
  }
}
