package com.team5817.lib.drivers.Actuator;

public class ActuatorIOLinear implements ActuatorIO {
  private final LinearActuator m_actuator;

  public ActuatorIOLinear(int pwmChannel, int minLength, int maxLength) {
    m_actuator = new LinearActuator(pwmChannel, minLength, maxLength);
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
