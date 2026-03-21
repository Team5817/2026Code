package com.team5817.frc2026.subsystems.Intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2026.Ports;
import com.team5817.lib.drivers.Rollers.IRollerState;
import com.team5817.lib.drivers.Rollers.RollerConstantsTalonFX;
import com.team5817.lib.drivers.Rollers.RollerSubsystem.RollerControlMode;
import com.team5817.lib.drivers.Servos.ServoConstants;
import lombok.Getter;

public class IntakeConstants {
  /** Constants related to the Intake Deploy subsystem. */
  public static final class DeployConstants {
    public static final ServoConstants kRackServoConstants = new ServoConstants();

    static {
      kRackServoConstants.kName = "Intake/Rack";

      kRackServoConstants.kMainConstants.id = Ports.INTAKE_DEPLOY;
      kRackServoConstants.kMainConstants.counterClockwisePositive = false;

      kRackServoConstants.kHomePosition = 0;
      kRackServoConstants.kRotationsPerUnitDistance = 41.15; // TODO or 143.6 * (3.28125 / 11.458)?
      kRackServoConstants.kMaxUnitsLimit = 0.3175;
      kRackServoConstants.kMinUnitsLimit = 0.0;

      kRackServoConstants.kKp = 15.0;
      kRackServoConstants.kKi = 0.0;
      kRackServoConstants.kKd = 0.08;
      kRackServoConstants.kKa = 0;
      kRackServoConstants.kKs = 0;

      kRackServoConstants.kKv = 0;
      kRackServoConstants.kKg = 0;

      kRackServoConstants.kCruiseVelocity = 1; // do .2 first

      kRackServoConstants.kAcceleration = 1000000000;

      kRackServoConstants.kMaxForwardOutput = 12.0;
      kRackServoConstants.kMaxReverseOutput = -12.0;

      kRackServoConstants.kEnableSupplyCurrentLimit = true;
      kRackServoConstants.kSupplyCurrentLimit = 80; // amps

      kRackServoConstants.kEnableStatorCurrentLimit = true;
      kRackServoConstants.kStatorCurrentLimit = 60; // amps

      kRackServoConstants.kNeutralMode = NeutralModeValue.Coast;

      kRackServoConstants.kHomingOutput = -.3;
      kRackServoConstants.kHomingTimeout = 0.2;
      kRackServoConstants.kHomingVelocityWindow = 5;
    }
  }

  public static final class RollerConstants {

    public static RollerConstantsTalonFX kMotorConstants = new RollerConstantsTalonFX();

    static {
      kMotorConstants.kSupplyCurrentLimit = 40;
      kMotorConstants.kStatorCurrentLimit = 80;
      kMotorConstants.kEnableSupplyCurrentLimit = true;
      kMotorConstants.kEnableStatorCurrentLimit = true;
      kMotorConstants.kMaxForwardOutput = 12.0;
      kMotorConstants.kMaxReverseOutput = -12.0;
    }

    public enum FeederState implements IRollerState {
      IDLE(0),
      INTAKING(-11),
      EXHAUST(11);

      @Getter private final double demand;
      @Getter private final RollerControlMode controlMode;

      FeederState(double demand) {
        this.demand = demand;
        this.controlMode = RollerControlMode.VOLTAGE;
      }
    }
  }
}
