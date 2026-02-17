package com.team5817.frc2026.subsystems.Climb;

import com.team5817.lib.drivers.Actuator.ActuatorIO;
import com.team5817.lib.drivers.Actuator.ActuatorSystem;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class LatchRelease extends ActuatorSystem {

  public LatchRelease(ActuatorIO io) {
    super(io, 1.0, "Climb/LatchRelease");
  }

  @Getter
  @Setter
  @Accessors(prefix = "m")
  private State mState = State.IDLE;

  public enum State {
    IDLE(0),
    RELEASED(0.5);

    private final double demand;

    State(double demand) {
      this.demand = demand;
    }

    public double getDemand() {
      return demand;
    }
  }

  @Override
  public void writePeriodicOutputs() {
    runPosition(mState.getDemand());
  }
}
