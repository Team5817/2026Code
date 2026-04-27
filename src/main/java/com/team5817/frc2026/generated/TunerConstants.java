package com.team5817.frc2026.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import edu.wpi.first.units.measure.*;

public class TunerConstants {

  private static final Slot0Configs steerGains = new Slot0Configs()
      .withKP(50)
      .withKI(0)
      .withKD(0.3)
      .withKS(0.1)
      .withKV(1.91);

  private static final Slot0Configs driveGains = new Slot0Configs()
      .withKP(0.1)
      .withKI(0)
      .withKD(0)
      .withKV(0.124);


  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // The type of motor used for the drive motor
  private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;

  // The type of motor used for the steer motor
  private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

  // The remote sensor feedback type to use for the steer motors;
  // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
  private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder;

  // === Current limits ===

  // The stator current at which the wheels start to slip;
  // This needs to be tuned to your individual robot
  private static final Current kSlipCurrent = Amps.of(120.0);

  // === Motor configs ===

  private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
          // Swerve azimuth does not require much torque output,
          // so we can set a relatively low stator current limit
          // to help avoid brownouts without impacting performance.
          .withStatorCurrentLimit(Amps.of(60))
          .withSupplyCurrentLimit(30)
          .withSupplyCurrentLimitEnable(true)
          .withStatorCurrentLimitEnable(true));


  private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
          // Swerve azimuth does not require much torque output,
          // so we can set a relatively low stator current limit
          // to help avoid brownouts without impacting performance.
          .withStatorCurrentLimit(Amps.of(60))
          .withSupplyCurrentLimit(30)
          .withSupplyCurrentLimitEnable(true)
          .withStatorCurrentLimitEnable(true));
     

  // Initial configs for the azimuth encoder
  private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

  // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
  private static final Pigeon2Configuration pigeonConfigs = null;

  // CAN bus that the swerve devices are located on;
  // All swerve modules must share the same CAN bus
  public static final CANBus kCANBus = new CANBus("canivore1");
  public static final CANBus Pigeon2Bus = new CANBus("canivore1");

  // === Drivetrain-wide co
  // Theoretical free speed (m/s) at 12 V applied output;
  // This needs to be tuned to your individual robot
  public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(20);

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  private static final double kCoupleRatio = 0;

  // Gear ratios (from module config)
  private static final double kDriveGearRatio = 5.68;
  private static final double kSteerGearRatio = 12.1;

  // Radius of the wheel
  private static final Distance kWheelRadius = Inches.of(2);

  // Inversion settings for drivetrain sides
  private static final boolean kInvertLeftSide = true;
  private static final boolean kInvertRightSide = false;

  // CAN ID for the Pigeon 2
  private static final int kPigeonId = 24;


  // Moment of inertia for steer and drive motors
  private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.004);
  private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.025);

  // Simulated voltage necessary to overcome friction
  private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
  private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

  // === Drivetrain config ===

  public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
      .withCANBusName(kCANBus.getName())
      .withPigeon2Id(kPigeonId)
      .withPigeon2Configs(pigeonConfigs);

  private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
      new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withCouplingGearRatio(kCoupleRatio)
          .withWheelRadius(kWheelRadius)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
          .withSlipCurrent(kSlipCurrent)
          .withSpeedAt12Volts(kSpeedAt12Volts)
          .withDriveMotorType(kDriveMotorType)
          .withSteerMotorType(kSteerMotorType)
          .withFeedbackSource(kSteerFeedbackType)
          .withDriveMotorInitialConfigs(driveInitialConfigs)
          .withSteerMotorInitialConfigs(steerInitialConfigs)
          .withEncoderInitialConfigs(encoderInitialConfigs)
          .withSteerInertia(kSteerInertia)
          .withDriveInertia(kDriveInertia)
          .withSteerFrictionVoltage(kSteerFrictionVoltage)
          .withDriveFrictionVoltage(kDriveFrictionVoltage);

  // === Module positions (based on 22.5" square wheelbase) ===

  private static final Distance kX = Inches.of(13.5);
  private static final Distance kY = Inches.of(13.5);

// === Module constants ===
// === Module constants (ROTATED 90° CCW: old RIGHT is now FRONT) ===
// Transform applied:
//   (x, y) -> ( y, -x )
//   steerZero -> steerZero - 0.25 rotations

public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
    ConstantCreator.createModuleConstants(
        5, 1, 1,
        Rotations.of(0.005126953125), // rotate steer zero
        kY.unaryMinus(),                                 // +y
        kX.unaryMinus(),                    // -x
        kInvertLeftSide, true, false);

public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
    ConstantCreator.createModuleConstants(
        6, 2, 2,
        Rotations.of(-0.104248046875), // rotate steer zero
        kY.unaryMinus(),                     // -y
        kX,                     // -x
        kInvertRightSide, true, false);

public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
    ConstantCreator.createModuleConstants(
        7, 3, 3,
        Rotations.of(-0.31298828125), // rotate steer zero
        kY,                                  // +y
        kX.unaryMinus(),                                  // +x
        kInvertLeftSide, true, false);

public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
    ConstantCreator.createModuleConstants(
        8, 4, 4,
        Rotations.of(-0.18212890625), // rotate steer zero
        kY,                     // -y
        kX,                                  // +x
        kInvertRightSide, true, false);

}
