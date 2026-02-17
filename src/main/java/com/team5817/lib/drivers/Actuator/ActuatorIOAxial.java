package com.team5817.lib.drivers.Actuator;

import edu.wpi.first.wpilibj.Servo;

public class ActuatorIOAxial implements ActuatorIO {
    private final Servo m_actuator;

    public ActuatorIOAxial(int pwmChannel) {
        m_actuator = new Servo(pwmChannel);
    }

    @Override
    public void runPosition(double units) {
        m_actuator.setPosition(units);
    }

    @Override
    public double getPosition() {
        return m_actuator.getPosition();
    }

    @Override
    public void updateInputs(ActuatorIOInputs inputs) {
        inputs.timestamp = System.currentTimeMillis();
        inputs.position_rotor = m_actuator.getPosition();
    }
}