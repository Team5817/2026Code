package com.team5817.frc2025.subsystems.Climb;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2025.Ports;
import com.team5817.lib.drivers.Servos.ServoConstants;
import com.team5817.lib.drivers.Servos.ServoMotorSubsystem.TalonFXConstants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Constants related to the Elevator subsystem.
 */
public final class ClimbConstants {
  // 115.93
  // 7.92
  public static final ServoConstants kClimbServoConstants = new ServoConstants();

  static {
    kClimbServoConstants.kName = "Climb";

    kClimbServoConstants.kMainConstants.id = Ports.CLIMB;
    kClimbServoConstants.kMainConstants.counterClockwisePositive = true;

    /*TalonFXConstants followerConstants = new TalonFXConstants();
    followerConstants.id = Ports.ELEVATOR_2;
    followerConstants.counterClockwisePositive = false;
    followerConstants.invert_sensor_phase = false;*/

   // kClimbServoConstants.kFollowerConstants = new TalonFXConstants[] { followerConstants };

    kClimbServoConstants.kHomePosition = 0; // degrees
    kClimbServoConstants.kRotationsPerUnitDistance = 72.82 / 1.4 * 3 / 4;

    kClimbServoConstants.kMaxUnitsLimit = 2.035;
    kClimbServoConstants.kMinUnitsLimit = 0.0;

    kClimbServoConstants.kKp = 15;
    kClimbServoConstants.kKi = 0.0;
    kClimbServoConstants.kKd = 0.2;
    kClimbServoConstants.kKa = 0.0;
    kClimbServoConstants.kKs = 0.0;
    kClimbServoConstants.kKv = .1;
    kClimbServoConstants.kKg = 7;

    kClimbServoConstants.kCruiseVelocity = 9999.0 / kClimbServoConstants.kRotationsPerUnitDistance; // degrees / s
    kClimbServoConstants.kAcceleration = 300 / kClimbServoConstants.kRotationsPerUnitDistance; // degrees / s^2

    kClimbServoConstants.kMaxForwardOutput = 12.0;
    kClimbServoConstants.kMaxReverseOutput = -12.0;

    kClimbServoConstants.kEnableSupplyCurrentLimit = true;
    kClimbServoConstants.kSupplyCurrentLimit = 40; // amps

    kClimbServoConstants.kEnableStatorCurrentLimit = true;
    kClimbServoConstants.kStatorCurrentLimit = 80; // amps

    kClimbServoConstants.kFollowerOpposeMasterDirection = true;

    kClimbServoConstants.kNeutralMode = NeutralModeValue.Brake;

    kClimbServoConstants.kHomingTimeout = 0.5;
    kClimbServoConstants.kHomingOutput = -.25;
    kClimbServoConstants.kHomingVelocityWindow = 0.1;

  }

  public static double kHomingZone = 0.1; // degrees
  public static final double kCoralClearHeight = 0.15; // rotations
  public static final double kCoralClearHeightRanThroughFinger = 1.6;
  public static final InterpolatingDoubleTreeMap kMidOffsetMap = new InterpolatingDoubleTreeMap();
  static {
    kMidOffsetMap.put(-.112, -0.149804);
    kMidOffsetMap.put(0.0, 0.0);
  }
  public static final InterpolatingDoubleTreeMap kHighOffsetMap = new InterpolatingDoubleTreeMap();
  static {
    kHighOffsetMap.put(-.11, -0.0);
    kHighOffsetMap.put(0.0, 0.0);
  }

}
