package com.team5817.lib.drivers.Actuator;

import edu.wpi.first.wpilibj.Timer;

public class ActuatorIOSim implements ActuatorIO {
    private double position = 0.0;
    private double velocity = 0.0;
    private double output = 0.0;

    @Override
    public void updateInputs(ActuatorIOInputs inputs) {
        // Simulate actuator behavior (for testing purposes)
        position += velocity * 0.02; // Assuming update is called every 20ms
        velocity += output * 0.1; // Simple acceleration model
        inputs.timestamp = Timer.getTimestamp();
        inputs.position_rotor = position;
    }

    @Override
    public void runPosition(double pos) {
        this.output = pos;
    }

    @Override
    public double getPosition() {
        return position;
    }
}
