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
    IDLE(() -> 0.0),
    CLOSE(() -> 60.0),
    FAR(() -> 80.0),
    SHOOT(() -> 95.0);

    private DoubleSupplier demand;

    FlywheelState(DoubleSupplier supplier) {
      this.demand = supplier;
    }

    public void setSupplier(DoubleSupplier supplier) {
      this.demand = supplier;
    }

    @Override
    public double getDemand() {
      return demand.getAsDouble();
    }

    @Override
    public double getToleranceRadsPerSec() {
      return 1.0;
    }

    @Override
    public RollerControlMode getControlMode() {
      return RollerControlMode.VELOCITY;
    }
  }


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
