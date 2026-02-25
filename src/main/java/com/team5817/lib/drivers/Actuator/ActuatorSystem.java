package com.team5817.lib.drivers.Actuator;

import com.team5817.lib.drivers.Subsystem;
import org.littletonrobotics.junction.Logger;

public abstract class ActuatorSystem extends Subsystem {
  private final ActuatorIO io;
  private final ActuatorIOInputsAutoLogged mInputs = new ActuatorIOInputsAutoLogged();
  private final double mSensorToUnits;
  private final String mName;

  protected ActuatorSystem(ActuatorIO io) {
    this(io, 1.0, "default");
  }

  protected ActuatorSystem(ActuatorIO io, double sensorToUnits, String name) {
    this.io = io;
    this.mSensorToUnits = sensorToUnits;
    this.mName = name;
  }

  @Override
  public void readPeriodicInputs() {
    io.updateInputs(mInputs);
    Logger.processInputs(mName, mInputs);
  }

  public void runPosition(double units) {
    io.runPosition(units);
  }

  public double getRotorPosition() {
    return mInputs.position_rotor;
  }

  public double getPosition() {
    return mInputs.position_rotor * mSensorToUnits;
  }
}
