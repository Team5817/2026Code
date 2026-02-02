package com.team5817.lib.drivers.Rollers;

import com.team5817.lib.drivers.Rollers.RollerSubsystem.RollerControlMode;

public interface IRollerState {
  public double getDemand();
  default double getFFVolts(){return 0;}

  default double getToleranceRadsPerSec() {
    return 0.0;
  }

  public RollerControlMode getControlMode();
}
