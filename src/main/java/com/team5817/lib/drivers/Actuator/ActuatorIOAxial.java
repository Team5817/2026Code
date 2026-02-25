package com.team5817.lib.drivers.Actuator;

import edu.wpi.first.wpilibj.Servo;

public class ActuatorIOAxial implements ActuatorIO {
  private final Servo m_actuator;
  public final double maxRange;

  public ActuatorIOAxial(int pwmChannel, double maxRange) {
    m_actuator = new Servo(pwmChannel);
    this.maxRange = maxRange;
  }

  public double toDegrees(double units) {
    return (units / maxRange);
  }

  @Override
  public void runPosition(double units) {
    m_actuator.setPosition(toDegrees(units));
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
