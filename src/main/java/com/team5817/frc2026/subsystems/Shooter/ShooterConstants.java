package com.team5817.frc2026.subsystems.Shooter;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team5817.frc2026.Ports;
import com.team5817.lib.drivers.Rollers.IRollerState;
import com.team5817.lib.drivers.Rollers.RollerConstantsTalonFX;
import com.team5817.lib.drivers.Rollers.RollerSubsystem.RollerControlMode;
import com.team5817.lib.drivers.Servos.ServoConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;

public class ShooterConstants {

  public static final RollerConstantsTalonFX kFlywheelConstants = new RollerConstantsTalonFX();
  public static final Translation3d TurretToCam;
  public static final Translation3d robotToTurret;
  public static double TURRET_YAW_SIGN = 1.0;
  public static double CAMERA_PITCH_DEGREES = -33;
  public static final InterpolatingDoubleTreeMap HOOD_MAP_LOB;
  public static final InterpolatingDoubleTreeMap FLYWHEEL_MAP_LOB;
  public static final InterpolatingDoubleTreeMap HOOD_MAP_HUB;
  public static final InterpolatingDoubleTreeMap FLYWHEEL_MAP_HUB;
  public static final double cancoderToTurretRatio = (360 / 400);

  public static Pose2d shooterTransform =
      new Pose2d(
          Units.inchesToMeters(-2.25), Units.inchesToMeters(4.625), Rotation2d.fromDegrees(0.0));

  static {
    robotToTurret =
        new Translation3d(
            Units.inchesToMeters(-2.25), Units.inchesToMeters(4.625), Units.inchesToMeters(21));
    TurretToCam = new Translation3d(Units.inchesToMeters(6.5), 0, 0);

    kFlywheelConstants.kMaxForwardOutput = 12.0;
    kFlywheelConstants.kMaxReverseOutput = -12.0;

    kFlywheelConstants.kNeutralMode = NeutralModeValue.Coast;
    kFlywheelConstants.kSupplyCurrentLimit = 40;
    kFlywheelConstants.kStatorCurrentLimit = 80;

    kFlywheelConstants.kKp = 0.04;
    kFlywheelConstants.kKs = 0.0703125;
    kFlywheelConstants.kKv = 0.008999999612569809;

    kFlywheelConstants.kEnableSupplyCurrentLimit = true;
    kFlywheelConstants.kEnableStatorCurrentLimit = true;

    kFlywheelConstants.counterClockwisePositive = false;
    kFlywheelConstants.kMainConstants.id = Ports.TURRET_FLYWHEEL1;
    kFlywheelConstants.kFollowerID = Ports.TURRET_FLYWHEEL2;
    kFlywheelConstants.kFollowerOpposeMasterDirection = true;

    // Default maps for LOB
    InterpolatingDoubleTreeMap lobHood = new InterpolatingDoubleTreeMap();
    lobHood.put(1.0, 20.0);
    lobHood.put(2.0, 22.0);
    lobHood.put(3.5, 28.0);
    lobHood.put(5.0, 31.0);

    InterpolatingDoubleTreeMap lobFly = new InterpolatingDoubleTreeMap();
    lobFly.put(6.0, 45.0);
    lobFly.put(10.0, 68.0);
    lobFly.put(15.0, 95.0);

    // Default maps for HUB
    InterpolatingDoubleTreeMap hubHood = new InterpolatingDoubleTreeMap();
    hubHood.put(1.0, 3.0);
    hubHood.put(2.64, 10.0);
    hubHood.put(3.5, 16.0);

    InterpolatingDoubleTreeMap hubFly = new InterpolatingDoubleTreeMap();
    hubFly.put(1.6, 42.0); // Front Hub
    hubFly.put(2.4, 47.5); // left hub
    hubFly.put(3.0, 51.5); // Mid Hub
    hubFly.put(3.6, 53.0); // depot
    hubFly.put(4.1, 54.5); // left mid wall
    hubFly.put(4.6, 56.0); // Human Side
    hubFly.put(5.2, 64.5); // Human Corner

    HOOD_MAP_LOB = lobHood;
    FLYWHEEL_MAP_LOB = lobFly;
    HOOD_MAP_HUB = hubHood;
    FLYWHEEL_MAP_HUB = hubFly;
  }

  public static final class TurretConstants {
    public static final ServoConstants kTurretServoConstants = new ServoConstants();

    static {
      kTurretServoConstants.kName = "Shooter/Turret";

      kTurretServoConstants.kMainConstants.id = Ports.TURRET;
      kTurretServoConstants.kMainConstants.counterClockwisePositive = false;

      kTurretServoConstants.kHomePosition = 0.0;
      kTurretServoConstants.kRotationsPerUnitDistance = 1 / 360.0 * 60;
      kTurretServoConstants.kMinUnitsLimit = -300.0;
      kTurretServoConstants.kMaxUnitsLimit = 0;

      kTurretServoConstants.kKp = 7.5;
      kTurretServoConstants.kKi = 0.0;
      kTurretServoConstants.kKd = 0.2;

      kTurretServoConstants.kKs = 0.0;
      kTurretServoConstants.kKv = 0.0;
      kTurretServoConstants.kKa = 0.0;
      kTurretServoConstants.kKg = 0.0;

      kTurretServoConstants.kGravityType = GravityTypeValue.Arm_Cosine;

      kTurretServoConstants.kMaxForwardOutput = 12.0;
      kTurretServoConstants.kMaxReverseOutput = -12.0;

      kTurretServoConstants.kEnableSupplyCurrentLimit = true;
      kTurretServoConstants.kSupplyCurrentLimit = 30;

      kTurretServoConstants.kEnableStatorCurrentLimit = true;
      kTurretServoConstants.kStatorCurrentLimit = 50;

      kTurretServoConstants.kNeutralMode = NeutralModeValue.Brake;

      kTurretServoConstants.kHomingTimeout = 0.5;
      kTurretServoConstants.kHomingOutput = -0.25;
      kTurretServoConstants.kHomingVelocityWindow = 1.0;
    }
  }

  public static final class HoodConstants {
    public static final ServoConstants kHoodServoConstants = new ServoConstants();

    static {
      kHoodServoConstants.kName = "Shooter/Hood";

      kHoodServoConstants.kMainConstants.id = Ports.HOOD;
      kHoodServoConstants.kMainConstants.counterClockwisePositive = false;

      kHoodServoConstants.kHomePosition = 0.0;
      kHoodServoConstants.kRotationsPerUnitDistance = 1 / 360.0 * 88.3; // og 1 / 360.0 * 96 / 1

      kHoodServoConstants.kMinUnitsLimit = 0.0;
      kHoodServoConstants.kMaxUnitsLimit = 30.0;

      kHoodServoConstants.kKp = 4.0;
      kHoodServoConstants.kKi = 7.0;
      kHoodServoConstants.kKd = 0.05;

      kHoodServoConstants.kKs = 0.0;
      kHoodServoConstants.kKv = 0.0;
      kHoodServoConstants.kKa = 0.0;
      kHoodServoConstants.kKg = 0.0;

      kHoodServoConstants.kGravityType = GravityTypeValue.Arm_Cosine;

      kHoodServoConstants.kMaxForwardOutput = 12.0;
      kHoodServoConstants.kMaxReverseOutput = -12.0;

      kHoodServoConstants.kEnableSupplyCurrentLimit = true;
      kHoodServoConstants.kSupplyCurrentLimit = 30;

      kHoodServoConstants.kEnableStatorCurrentLimit = true;
      kHoodServoConstants.kStatorCurrentLimit = 15;

      kHoodServoConstants.kNeutralMode = NeutralModeValue.Coast;

      kHoodServoConstants.kHomingTimeout = 0.5;
      kHoodServoConstants.kHomingOutput = -0.2;
      kHoodServoConstants.kHomingVelocityWindow = 1.0;
    }
  }

  public enum FlywheelState implements IRollerState {
    IDLE(15, RollerControlMode.VOLTAGE),
    CLOSE(43.0, RollerControlMode.VELOCITY),
    FAR(65.0, RollerControlMode.VELOCITY),
    HUB(70.0, RollerControlMode.VELOCITY),
    LOBBING(80.0, RollerControlMode.VELOCITY);

    private final RollerControlMode controlMode;
    private final double toleranceRadsPerSec = 0.1;
    private DoubleSupplier supplier = () -> 0.0;

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
}
