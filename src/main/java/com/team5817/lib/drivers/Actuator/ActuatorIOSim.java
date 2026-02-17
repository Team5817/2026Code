package com.team5817.lib.drivers.Actuator;

import edu.wpi.first.wpilibj.Timer;

public class ActuatorIOSim implements ActuatorIO {

    private double output = 0.0;

    @Override
    public void updateInputs(ActuatorIOInputs inputs) {
        inputs.timestamp = Timer.getTimestamp();
        inputs.position_rotor = output;
    }

    @Override
    public void runPosition(double pos) {
        this.output = pos;
    }

    @Override
    public double getPosition() {
        return output;
    }
}
