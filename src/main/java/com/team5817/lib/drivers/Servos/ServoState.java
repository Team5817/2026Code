package com.team5817.lib.drivers.Servos;

public interface ServoState {
  double getDemand();

  boolean isDisabled();

  public default double getAllowableError() {
    return Double.POSITIVE_INFINITY;
  }

  ServoMotorSubsystem.ControlState getControlState();
}
