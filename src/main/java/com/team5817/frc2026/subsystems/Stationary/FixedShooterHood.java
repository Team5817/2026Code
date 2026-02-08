package com.team5817.frc2026.subsystems.Stationary;

import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class FixedShooterHood
    extends StateBasedServoMotorSubsystem<FixedShooterHood.State> {

  private static final double kTightError = 1.5;
  private static final double kLooseError = 4.0;

  public FixedShooterHood(
      ServoMotorIO io,
      DoubleSupplier closeAngleSupplier,
      DoubleSupplier farAngleSupplier) {

    super(State.STOW, io);

    State.CLOSE.setSupplier(closeAngleSupplier);
    State.FAR.setSupplier(farAngleSupplier);
  }

  public enum State implements ServoState {
    STOW(() -> 0.0, kLooseError),
    CLOSE(kTightError),
    FAR(kTightError);

    private DoubleSupplier demand;
    @Getter private final double allowableError;

    State(double allowableError) {
      this.demand = () -> 0.0;
      this.allowableError = allowableError;
    }

    State(DoubleSupplier supplier, double allowableError) {
      this.demand = supplier;
      this.allowableError = allowableError;
    }

    void setSupplier(DoubleSupplier supplier) {
      this.demand = supplier;
    }

    @Override
    public double getDemand() {
      return demand.getAsDouble();
    }

    @Override
    public ControlState getControlState() {
      return ControlState.POSITION;
    }

    @Override
    public boolean isDisabled() {
      return false;
    }
  }
}
