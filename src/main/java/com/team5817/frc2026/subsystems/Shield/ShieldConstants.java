package com.team5817.frc2026.subsystems.Shield;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2026.Ports;
import com.team5817.lib.drivers.Servos.ServoConstants;

public final class ShieldConstants {
  public static final ServoConstants kShieldServoConstants = new ServoConstants();

  static {
    kShieldServoConstants.kName = "Shield";

    kShieldServoConstants.kMainConstants.id = Ports.SHIELD;
    kShieldServoConstants.kMainConstants.counterClockwisePositive = true;

    kShieldServoConstants.kHomePosition = 0; // meters

    kShieldServoConstants.kRotationsPerUnitDistance = 8.09 / .302;

    kShieldServoConstants.kMaxUnitsLimit = .302;
    kShieldServoConstants.kMinUnitsLimit = 0.0;

    kShieldServoConstants.kKp = 10.0;
    kShieldServoConstants.kKi = 0.0;
    kShieldServoConstants.kKd = 0.0;
    kShieldServoConstants.kKa = 0.0;
    kShieldServoConstants.kKs = 0.0;
    kShieldServoConstants.kKv = 0.0;
    kShieldServoConstants.kKg = 2.0;

    kShieldServoConstants.kCruiseVelocity = 15.0 / kShieldServoConstants.kRotationsPerUnitDistance;
    kShieldServoConstants.kAcceleration = 999 / kShieldServoConstants.kRotationsPerUnitDistance;

    kShieldServoConstants.kMaxForwardOutput = 12.0;
    kShieldServoConstants.kMaxReverseOutput = -12.0;

    kShieldServoConstants.kEnableSupplyCurrentLimit = true;
    kShieldServoConstants.kSupplyCurrentLimit = 30; // amps

    kShieldServoConstants.kEnableStatorCurrentLimit = true;
    kShieldServoConstants.kStatorCurrentLimit = 60; // amps

    kShieldServoConstants.kNeutralMode = NeutralModeValue.Brake;

    kShieldServoConstants.kHomingTimeout = 0.5;
    kShieldServoConstants.kHomingOutput = -.25;
    kShieldServoConstants.kHomingVelocityWindow = 0.1;
  }
}
