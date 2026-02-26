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

      kRackServoConstants.kMainConstants.id = Ports.RACK;
      kRackServoConstants.kMainConstants.counterClockwisePositive = false;

      kRackServoConstants.kHomePosition = 0;
      kRackServoConstants.kRotationsPerUnitDistance = 143.6;

      kRackServoConstants.kMaxUnitsLimit = 0.2667;
      kRackServoConstants.kMinUnitsLimit = 0.0;

      kRackServoConstants.kKp = 3.8125;
      kRackServoConstants.kKi = 0.0;
      kRackServoConstants.kKd = 0;
      kRackServoConstants.kKa = 0;
      kRackServoConstants.kKs = 0;
      kRackServoConstants.kKv = .5;
      kRackServoConstants.kKg = 0.265625;

      kRackServoConstants.kCruiseVelocity = 1;

      kRackServoConstants.kAcceleration = 1;

      kRackServoConstants.kMaxForwardOutput = 12.0;
      kRackServoConstants.kMaxReverseOutput = -12.0;

      kRackServoConstants.kEnableSupplyCurrentLimit = true;
      kRackServoConstants.kSupplyCurrentLimit = 80; // amps

      kRackServoConstants.kEnableStatorCurrentLimit = true;
      kRackServoConstants.kStatorCurrentLimit = 80; // amps

      kRackServoConstants.kNeutralMode = NeutralModeValue.Brake;

      kRackServoConstants.kHomingOutput = -.3;
      kRackServoConstants.kHomingTimeout = 0.2;
      kRackServoConstants.kHomingVelocityWindow = 5;
    }
  }

  public static final class RollerConstants {

    public static RollerConstantsTalonFX motorConstants = new RollerConstantsTalonFX();

    static {
      motorConstants.kSupplyCurrentLimit = 40;
      motorConstants.kStatorCurrentLimit = 80;
      motorConstants.kEnableSupplyCurrentLimit = true;
      motorConstants.kEnableStatorCurrentLimit = true;
      motorConstants.kMaxForwardOutput = 12.0;
      motorConstants.kMaxReverseOutput = -12.0;
    }

    public enum FeederState implements IRollerState {
      IDLE(0),
      INTAKING(11),
      EXHAUST(-12);

      @Getter private final double demand;
      @Getter private final RollerControlMode controlMode;

      FeederState(double demand) {
        this.demand = demand;
        this.controlMode = RollerControlMode.VOLTAGE;
      }
    }
  }
}
