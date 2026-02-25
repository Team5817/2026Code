package com.team5817.lib.drivers.Actuator;

import org.littletonrobotics.junction.AutoLog;

public interface ActuatorIO {
  public default void runPosition(double units) {}

  public default double getPosition() {
    return 0;
  }

  @AutoLog
  public static class ActuatorIOInputs {
    public double timestamp;
    public double position_rotor;
  }

  public default void updateInputs(ActuatorIOInputs inputs) {}
}
