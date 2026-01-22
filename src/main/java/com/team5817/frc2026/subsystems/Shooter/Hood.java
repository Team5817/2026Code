package com.team5817.frc2026.subsystems.Shooter;

import com.team5817.frc2026.RobotVisualizer;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class Hood extends StateBasedServoMotorSubsystem<Hood.State> {

  private static final double kTightError = 1.3;
  private static final double kLooseError = 4.0;

  public Hood(ServoMotorIO io, DoubleSupplier hubAngleSupplier, DoubleSupplier lobAngleSupplier) {
    super(State.STOW, io);

    State.HUB.setSupplier(hubAngleSupplier);
    State.LOBBING.setSupplier(lobAngleSupplier);
  }

  public enum State implements ServoState {
    STOW(() -> 0.0, kLooseError),
    CLOSE(() -> 0.0, kTightError),
    FAR(() -> 0.0, kTightError),
    HUB(kTightError),
    LOBBING(kTightError);

    private DoubleSupplier demand;
    @Getter private final double allowableError;

    State(double allowableError) {
      this.demand = () -> 0;
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
    public boolean isDisabled() {
      return false;
    }

    @Override
    public ControlState getControlState() {
      return ControlState.POSITION;
    }
  }
    @Override
  public void outputTelemetry() {
    RobotVisualizer.updateHoodAngle(getPosition());

    super.outputTelemetry();
  }
}
