package com.team5817.lib.drivers.Actuator;

import com.team5817.lib.drivers.Subsystem;

public abstract class ActuatorSystem extends Subsystem {
    private final ActuatorIO io;
    private final ActuatorIOInputsAutoLogged mInputs = new ActuatorIOInputsAutoLogged();
    private final double mSensorToUnits;

    protected ActuatorSystem(ActuatorIO io) {
        this(io, 1.0);
    }

    protected ActuatorSystem(ActuatorIO io, double sensorToUnits) {
        this.io = io;
        this.mSensorToUnits = sensorToUnits;
    }

   @Override
   public void readPeriodicInputs() {
       io.updateInputs(mInputs);
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
