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
import com.team5817.lib.drivers.Servos.ServoMotorSubsystem.TalonFXConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

import java.util.function.DoubleSupplier;

public class ShooterConstants {

  public static final RollerConstantsTalonFX flywheelConstants = new RollerConstantsTalonFX();
  public static final Translation3d TurretToCam;
  public static final Translation3d robotToTurret;
  // Allows quick sign flip when turret yaw axis convention differs (1.0 or -1.0)
  public static double TURRET_YAW_SIGN = 1.0;
  // Camera pitch in degrees (positive = nose-up). Adjust to match the physical mount.
  public static double CAMERA_PITCH_DEGREES = -45;
  public static final InterpolatingDoubleTreeMap HOOD_MAP_LOB;
  public static final InterpolatingDoubleTreeMap FLYWHEEL_MAP_LOB;
  public static final InterpolatingDoubleTreeMap HOOD_MAP_HUB;
  public static final InterpolatingDoubleTreeMap FLYWHEEL_MAP_HUB;
  
  public static Pose2d shooterTransform =
  new Pose2d(
    0.0, 0.0, Rotation2d.fromDegrees(0.0)); // TODO: Adjust based on actual robot geometry
    
    static {
    robotToTurret = new Translation3d(Units.inchesToMeters(6.5),0,Units.inchesToMeters(21.5));
    TurretToCam = new Translation3d(Units.inchesToMeters(6.5),0,0);

    // Basic flywheel/talon defaults
    flywheelConstants.kMaxForwardOutput = 12.0;
    flywheelConstants.kMaxReverseOutput = -12.0;

    flywheelConstants.kNeutralMode = NeutralModeValue.Coast;
    flywheelConstants.kSupplyCurrentLimit = 40;
    flywheelConstants.kStatorCurrentLimit = 80;

    flywheelConstants.kKp = 0.3;
    flywheelConstants.kKs = 0.7;
    flywheelConstants.kKv = 0.109;

    flywheelConstants.kEnableSupplyCurrentLimit = true;
    flywheelConstants.kEnableStatorCurrentLimit = true;

    flywheelConstants.counterClockwisePositive = true;

    // Follower Motor
    TalonFXConstants followerConstants = new TalonFXConstants();
    followerConstants.id = Ports.FLYWHEEL_2;
    followerConstants.counterClockwisePositive = false;
    followerConstants.invert_sensor_phase = false;
    flywheelConstants.kFollowerConstants = new TalonFXConstants[] {followerConstants};

    flywheelConstants.kFollowerOpposeMasterDirection = false;

    // Default maps for LOB
    InterpolatingDoubleTreeMap lobHood = new InterpolatingDoubleTreeMap();
    lobHood.put(1.0, 10.0);
    lobHood.put(2.0, 12.5);
    lobHood.put(3.5, 15.0);
    lobHood.put(5.0, 18.0);

    InterpolatingDoubleTreeMap lobFly = new InterpolatingDoubleTreeMap();
    lobFly.put(1.0, 80.0);
    lobFly.put(5.0, 90.0);

    // Default maps for HUB
    InterpolatingDoubleTreeMap hubHood = new InterpolatingDoubleTreeMap();
    hubHood.put(1.0, 5.0);
    hubHood.put(2.0, 7.5);
    hubHood.put(3.5, 10.0);
    hubHood.put(5.0, 12.0);

    InterpolatingDoubleTreeMap hubFly = new InterpolatingDoubleTreeMap();
    hubFly.put(1.0, 50.0);
    hubFly.put(5.0, 80.0);

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
      kTurretServoConstants.kMainConstants.counterClockwisePositive = true;

      kTurretServoConstants.kHomePosition = 0.0;
      kTurretServoConstants.kRotationsPerUnitDistance = 1 / 360.0 * 100 / 3 * 1.19444444444444;

      // Soft limits
      kTurretServoConstants.kMinUnitsLimit = -180.0;
      kTurretServoConstants.kMaxUnitsLimit = 180.0;

      // PID (placeholder)
      kTurretServoConstants.kKp = 4.0;
      kTurretServoConstants.kKi = 5.0;
      kTurretServoConstants.kKd = 0.1;

      kTurretServoConstants.kKs = 0.0;
      kTurretServoConstants.kKv = 0.0;
      kTurretServoConstants.kKa = 0.0;
      kTurretServoConstants.kKg = 0.0;

      kTurretServoConstants.kGravityType = GravityTypeValue.Arm_Cosine;

      kTurretServoConstants.kCruiseVelocity = 200000;
      kTurretServoConstants.kAcceleration = 10000;

      kTurretServoConstants.kMaxForwardOutput = 12.0;
      kTurretServoConstants.kMaxReverseOutput = -12.0;

      kTurretServoConstants.kEnableSupplyCurrentLimit = true;
      kTurretServoConstants.kSupplyCurrentLimit = 30;

      kTurretServoConstants.kEnableStatorCurrentLimit = true;
      kTurretServoConstants.kStatorCurrentLimit = 30;

      kTurretServoConstants.kNeutralMode = NeutralModeValue.Brake;

      // homing
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
      kHoodServoConstants.kRotationsPerUnitDistance = 1 / 360.0 * 100 / 1;

      // Soft limits
      kHoodServoConstants.kMinUnitsLimit = 0.0;
      kHoodServoConstants.kMaxUnitsLimit = 43.0;

      // PID (placeholder)
      kHoodServoConstants.kKp = 20.0;
      kHoodServoConstants.kKi = 9.5;
      kHoodServoConstants.kKd = 0.4;

      kHoodServoConstants.kKs = 0.9;
      kHoodServoConstants.kKv = 0.0;
      kHoodServoConstants.kKa = 0.0;
      kHoodServoConstants.kKg = 0.0;

      kHoodServoConstants.kGravityType = GravityTypeValue.Arm_Cosine;

      kHoodServoConstants.kCruiseVelocity = 200000;
      kHoodServoConstants.kAcceleration = 10000;

      kHoodServoConstants.kMaxForwardOutput = 12.0;
      kHoodServoConstants.kMaxReverseOutput = -12.0;

      kHoodServoConstants.kEnableSupplyCurrentLimit = true;
      kHoodServoConstants.kSupplyCurrentLimit = 30;

      kHoodServoConstants.kEnableStatorCurrentLimit = true;
      kHoodServoConstants.kStatorCurrentLimit = 15;

      kHoodServoConstants.kNeutralMode = NeutralModeValue.Brake;

      kHoodServoConstants.kHomingTimeout = 0.5;
      kHoodServoConstants.kHomingOutput = -0.2;
      kHoodServoConstants.kHomingVelocityWindow = 1.0;
    }
  }

  public enum FlywheelState implements IRollerState {
    IDLE(0.0, RollerControlMode.VOLTAGE),
    CLOSE(50.0, RollerControlMode.VELOCITY),
    FAR(80.0, RollerControlMode.VELOCITY),
    HUB(70.0, RollerControlMode.VELOCITY), // Placeholder, set at runtime
    LOBBING(80.0, RollerControlMode.VELOCITY); // Placeholder, set at runtime

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
