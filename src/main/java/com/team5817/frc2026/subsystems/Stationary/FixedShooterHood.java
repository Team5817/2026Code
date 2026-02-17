package com.team5817.frc2026.subsystems.Stationary;

import com.team5817.frc2026.subsystems.Stationary.FixedShooterConstants.FixedShooterHoodConstants;
import com.team5817.lib.drivers.Actuator.ActuatorIO;
import com.team5817.lib.drivers.Actuator.ActuatorSystem;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;

public class FixedShooterHood extends ActuatorSystem {

  private static final double kTightError = 1.5;
  private static final double kLooseError = 4.0;

  public FixedShooterHood(
      ActuatorIO io,
      DoubleSupplier hubAngleSupplier,
      DoubleSupplier lobAngleSupplier) {

    super(io, FixedShooterHoodConstants.kSensorToDegrees,"FixedShooter/Hood");

    State.HUB.setSupplier(hubAngleSupplier);
    State.LOB.setSupplier(lobAngleSupplier);
  }

  @Getter @Setter @Accessors(prefix = "m")
  private State mState = State.STOW;

  public enum State {
    STOW(() -> 0.0, kLooseError),
    CLOSE(() -> 10.0, kTightError),
    FAR(() -> 18.0, kTightError),
    HUB(() -> 0.0, kTightError),
    LOB(() -> 0.0, kTightError);

    private DoubleSupplier demand;
    @Getter private final double allowableError;

    State(DoubleSupplier supplier, double allowableError) {
      this.demand = supplier;
      this.allowableError = allowableError;
    }

    void setSupplier(DoubleSupplier supplier) {
      this.demand = supplier;
    }

    public double getDemand() {
      return demand.getAsDouble();
    }
  }

  @Override
  public void writePeriodicOutputs() {
    runPosition(mState.getDemand());
  }
  public boolean atState() {
    return Math.abs(getPosition() - mState.getDemand()) < mState.getAllowableError();
  }
}