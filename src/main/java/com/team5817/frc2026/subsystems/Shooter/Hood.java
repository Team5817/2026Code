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

  public Hood(
      ServoMotorIO io,
      DoubleSupplier hubAngleSupplier,
      DoubleSupplier hubVelocityFFSupplier,
      DoubleSupplier lobAngleSupplier,
      DoubleSupplier lobVelocityFFSupplier) {
    super(State.STOW, io);

    State.HUB.setDemandSupplier(hubAngleSupplier);
    State.HUB.setFFSupplier(hubVelocityFFSupplier);
    State.LOBBING.setDemandSupplier(lobAngleSupplier);
    State.LOBBING.setFFSupplier(lobVelocityFFSupplier);
  }

  public enum State implements ServoState {
    STOW(() -> 0.0, () -> 0.0, kLooseError),
    CLOSE(() -> 0.0, () -> 0.0, kTightError),
    FAR(() -> 0.0, () -> 0.0, kTightError),
    HUB(null, null, kTightError),
    LOBBING(null, null, kTightError);

    private DoubleSupplier demandSupplier;
    private DoubleSupplier ffSupplier;
    @Getter private final double allowableError;

    State(DoubleSupplier demandSupplier, DoubleSupplier ffSupplier, double allowableError) {
      this.demandSupplier = demandSupplier != null ? demandSupplier : () -> 0.0;
      this.ffSupplier = ffSupplier != null ? ffSupplier : () -> 0.0;
      this.allowableError = allowableError;
    }

    public void setDemandSupplier(DoubleSupplier supplier) {
      if (supplier != null) {
        this.demandSupplier = supplier;
      }
    }

    public void setFFSupplier(DoubleSupplier supplier) {
      if (supplier != null) {
        this.ffSupplier = supplier;
      }
    }

    @Override
    public double getDemand() {
      return demandSupplier.getAsDouble();
    }

    public double getFF() {
      return ffSupplier.getAsDouble();
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
