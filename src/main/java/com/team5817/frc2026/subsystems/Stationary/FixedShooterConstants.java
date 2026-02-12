package com.team5817.frc2026.subsystems.Stationary;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2026.Ports;
import com.team5817.lib.drivers.Rollers.IRollerState;
import com.team5817.lib.drivers.Rollers.RollerConstantsTalonFX;
import com.team5817.lib.drivers.Rollers.RollerSubsystem.RollerControlMode;
import com.team5817.lib.drivers.Servos.ServoConstants;
import com.team5817.lib.drivers.Servos.ServoMotorSubsystem.TalonFXConstants;
import java.util.function.DoubleSupplier;

public class FixedShooterConstants {

  /* ================= FLYWHEEL ================= */

  public static final RollerConstantsTalonFX flywheelConstants =
      new RollerConstantsTalonFX();

  static {
    flywheelConstants.kMaxForwardOutput = 12.0;
    flywheelConstants.kMaxReverseOutput = -12.0;
    flywheelConstants.kNeutralMode = NeutralModeValue.Coast;

    flywheelConstants.kEnableSupplyCurrentLimit = true;
    flywheelConstants.kSupplyCurrentLimit = 40;
    flywheelConstants.kEnableStatorCurrentLimit = true;
    flywheelConstants.kStatorCurrentLimit = 80;

    flywheelConstants.counterClockwisePositive = false;

    TalonFXConstants follower = new TalonFXConstants();
    follower.id = Ports.FLYWHEEL_2;
    follower.counterClockwisePositive = false;

    flywheelConstants.kFollowerConstants = new TalonFXConstants[] {follower};
    flywheelConstants.kFollowerOpposeMasterDirection = false;
  }

public enum FlywheelState implements IRollerState {
  IDLE(0.0, RollerControlMode.VOLTAGE),
  CLOSE(60.0, RollerControlMode.VELOCITY),
  FAR(80.0, RollerControlMode.VELOCITY),
  HUB(70.0, RollerControlMode.VELOCITY), // Placeholder
  LOBBING(90.0, RollerControlMode.VELOCITY); // Placeholder

  private final RollerControlMode controlMode;
  private final double toleranceRadsPerSec = 1.0;
  private DoubleSupplier supplier;

  FlywheelState(double demand, RollerControlMode controlMode) {
    this.controlMode = controlMode;
    this.supplier = () -> demand;
  }

  public void setSupplier(DoubleSupplier supplier) {
    this.supplier = supplier == null ? () -> 0.0 : supplier;
  }

  @Override
  public double getDemand() {
    return supplier.getAsDouble();
  }

  @Override
  public double getToleranceRadsPerSec() {
    return toleranceRadsPerSec;
  }

  @Override
  public RollerControlMode getControlMode() {
    return controlMode;
  }
}


  /* ================= HOOD ================= */

  public static final class FixedShooterHoodConstants {
    public static final ServoConstants kHoodServoConstants = new ServoConstants();

    static {
      kHoodServoConstants.kName = "FixedShooter/Hood";
      kHoodServoConstants.kMainConstants.id = Ports.HOOD;
      kHoodServoConstants.kMainConstants.counterClockwisePositive = true;

      kHoodServoConstants.kRotationsPerUnitDistance = 1.0 / 360.0;
      kHoodServoConstants.kMinUnitsLimit = 0.0;
      kHoodServoConstants.kMaxUnitsLimit = 60.0;

      kHoodServoConstants.kKp = 1.2;
      kHoodServoConstants.kKi = 0.0;
      kHoodServoConstants.kKd = 0.0;

      kHoodServoConstants.kMaxForwardOutput = 12.0;
      kHoodServoConstants.kMaxReverseOutput = -12.0;
      kHoodServoConstants.kNeutralMode = NeutralModeValue.Brake;

      kHoodServoConstants.kHomingTimeout = 0.5;
      kHoodServoConstants.kHomingOutput = -0.2;
      kHoodServoConstants.kHomingVelocityWindow = 1.0;
    }
  }
}
