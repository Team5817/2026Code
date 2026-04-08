package com.team5817.lib.drivers.Servos;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.lib.drivers.Servos.ServoMotorSubsystem.TalonFXConstants;

public class ServoConstants {
  public String kName = "ERROR_ASSIGN_A_NAME";

  public double kLooperDt = 0.01;
  public double kCANTimeout = 0.010; // use for important on the fly updates
  public int kLongCANTimeoutMs = 100; // use for constructors

  public TalonFXConstants kMainConstants = new TalonFXConstants();
  public TalonFXConstants[] kFollowerConstants = new TalonFXConstants[0];

  public GravityTypeValue kGravityType = GravityTypeValue.Elevator_Static;
  public NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
  public double kHomePosition = 0.0; // Units
  public double kRotationsPerUnitDistance = 1.0;
  public double kSoftLimitDeadband = 0.0;
  public double kKp = 0; // Raw output / raw error
  public double kKi = 0; // Raw output / sum of raw error
  public double kKd = 0; // Raw output / (err - prevErr)
  public double kKv = 0;
  public double kKa = 0;
  public double kKs = 0;
  public double kKg = 0;
  public int kDeadband = 0; // rotation

  public double kPositionKp = 0;
  public double kPositionKi = 0;
  public double kPositionKd = 0;

  public double kVelocityFeedforward = 0;
  public double kArbitraryFeedforward = 0;
  public double kCruiseVelocity = 0; // Units/s
  public double kAcceleration = 0; // Units/s^2
  public double kJerk = 0; // Units/s^3
  public double kRampRate = 0.0; // s

  public int kSupplyCurrentLimit = 60; // amps
  public boolean kEnableSupplyCurrentLimit = false;

  public int kStatorCurrentLimit = 40; // amps
  public boolean kEnableStatorCurrentLimit = false;

  public double kMaxForwardOutput = 12.0; // Volts
  public double kMaxReverseOutput = -12.0; // Voltsa

  public MotorAlignmentValue kFollowerOpposeMasterDirection = MotorAlignmentValue.Aligned;

  public double kMaxUnitsLimit = Double.POSITIVE_INFINITY;
  public double kMinUnitsLimit = Double.NEGATIVE_INFINITY;

  public double kHomingTimeout = 0;
  public double kHomingVelocityWindow = 0;
  public double kHomingOutput = 0;

  /**
   * Converts rotations to units.
   *
   * @param rotations The rotations.
   * @return The units.
   */
  protected double rotationsToUnits(double rotations) {
    return rotations / kRotationsPerUnitDistance;
  }

  /**
   * Converts units to rotations.
   *
   * @param units The units.
   * @return The rotations.
   */
  protected double unitsToRotations(double units) {
    return units * kRotationsPerUnitDistance;
  }

  public double getForwardSoftLimitRotations() {
    return (((kMaxUnitsLimit) * kRotationsPerUnitDistance) - kSoftLimitDeadband);
  }

  public double getReverseSoftLimitRotations() {
    return (((kMinUnitsLimit) * kRotationsPerUnitDistance) + kSoftLimitDeadband);
  }

  public ServoConstants copy() {
    ServoConstants copy = new ServoConstants();
    copy.kName = this.kName;
    copy.kLooperDt = this.kLooperDt;
    copy.kCANTimeout = this.kCANTimeout;
    copy.kLongCANTimeoutMs = this.kLongCANTimeoutMs;
    copy.kMainConstants = this.kMainConstants;
    copy.kFollowerConstants = this.kFollowerConstants;
    copy.kGravityType = this.kGravityType;
    copy.kNeutralMode = this.kNeutralMode;
    copy.kHomePosition = this.kHomePosition;
    copy.kRotationsPerUnitDistance = this.kRotationsPerUnitDistance;
    copy.kSoftLimitDeadband = this.kSoftLimitDeadband;
    copy.kKp = this.kKp;
    copy.kKi = this.kKi;
    copy.kKd = this.kKd;
    copy.kKv = this.kKv;
    copy.kKa = this.kKa;
    copy.kKs = this.kKs;
    copy.kKg = this.kKg;
    copy.kDeadband = this.kDeadband;
    copy.kPositionKp = this.kPositionKp;
    copy.kPositionKi = this.kPositionKi;
    copy.kPositionKd = this.kPositionKd;
    copy.kVelocityFeedforward = this.kVelocityFeedforward;
    copy.kArbitraryFeedforward = this.kArbitraryFeedforward;
    copy.kCruiseVelocity = this.kCruiseVelocity;
    copy.kAcceleration = this.kAcceleration;
    copy.kJerk = this.kJerk;
    copy.kRampRate = this.kRampRate;
    copy.kSupplyCurrentLimit = this.kSupplyCurrentLimit;
    copy.kEnableSupplyCurrentLimit = this.kEnableSupplyCurrentLimit;
    copy.kStatorCurrentLimit = this.kStatorCurrentLimit;
    copy.kEnableStatorCurrentLimit = this.kEnableStatorCurrentLimit;
    copy.kMaxForwardOutput = this.kMaxForwardOutput;
    copy.kMaxReverseOutput = this.kMaxReverseOutput;
    copy.kFollowerOpposeMasterDirection = this.kFollowerOpposeMasterDirection;
    copy.kMaxUnitsLimit = this.kMaxUnitsLimit;
    copy.kMinUnitsLimit = this.kMinUnitsLimit;
    copy.kHomingTimeout = this.kHomingTimeout;
    copy.kHomingVelocityWindow = this.kHomingVelocityWindow;
    copy.kHomingOutput = this.kHomingOutput;
    return copy;
  }
}
