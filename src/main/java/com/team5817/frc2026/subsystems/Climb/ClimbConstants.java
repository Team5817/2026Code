package com.team5817.frc2026.subsystems.Climb;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2026.Ports;
import com.team5817.lib.drivers.Servos.ServoConstants;

public final class ClimbConstants {
  public static final ServoConstants kClimbServoConstants = new ServoConstants();

  static {
    kClimbServoConstants.kName = "Climb";

    kClimbServoConstants.kMainConstants.id = Ports.CLIMB;
    kClimbServoConstants.kMainConstants.counterClockwisePositive = false;

    kClimbServoConstants.kHomePosition = 0; // degrees
    kClimbServoConstants.kRotationsPerUnitDistance = -129.205/90;

    kClimbServoConstants.kMaxUnitsLimit = 140;
    kClimbServoConstants.kMinUnitsLimit = 0.0;

    kClimbServoConstants.kKp = 0;//16;
    kClimbServoConstants.kKi = 0.0;
    kClimbServoConstants.kKd = 0;//0.2;
    kClimbServoConstants.kKa = 0.0;
    kClimbServoConstants.kKs = 0.0;
    kClimbServoConstants.kKv = 0.0;
    kClimbServoConstants.kKg = 0.0;

    kClimbServoConstants.kCruiseVelocity =
        9999.0 / kClimbServoConstants.kRotationsPerUnitDistance; // degrees / s
    kClimbServoConstants.kAcceleration =
        300 / kClimbServoConstants.kRotationsPerUnitDistance; // degrees / s^2

    kClimbServoConstants.kMaxForwardOutput = 12.0;
    kClimbServoConstants.kMaxReverseOutput = -12.0;

    kClimbServoConstants.kEnableSupplyCurrentLimit = true;
    kClimbServoConstants.kSupplyCurrentLimit = 40; // amps

    kClimbServoConstants.kEnableStatorCurrentLimit = true;
    kClimbServoConstants.kStatorCurrentLimit = 80; // amps

    kClimbServoConstants.kFollowerOpposeMasterDirection = MotorAlignmentValue.Aligned;

    kClimbServoConstants.kNeutralMode = NeutralModeValue.Coast;

    kClimbServoConstants.kHomingTimeout = 0.5;
    kClimbServoConstants.kHomingOutput = -.25;
    kClimbServoConstants.kHomingVelocityWindow = 0.1;
  }
}
