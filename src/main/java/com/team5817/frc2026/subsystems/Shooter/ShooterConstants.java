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

  public static final RollerConstantsTalonFX kFlywheelConstants = new RollerConstantsTalonFX();
  public static final Translation3d TurretToCam;
  public static final Translation3d robotToTurret;
  // Allows quick sign flip when turret yaw axis convention differs (1.0 or -1.0)
  public static double TURRET_YAW_SIGN = 1.0;
  // Camera pitch in degrees (positive = nose-up). Adjust to match the physical mount.
  public static double CAMERA_PITCH_DEGREES = -33;
  public static final InterpolatingDoubleTreeMap HOOD_MAP_LOB;
  public static final InterpolatingDoubleTreeMap FLYWHEEL_MAP_LOB;
  public static final InterpolatingDoubleTreeMap HOOD_MAP_HUB;
  public static final InterpolatingDoubleTreeMap FLYWHEEL_MAP_HUB;
  public static final double cancoderToTurretRatio = (360 / 400); // TODO

  public static Pose2d shooterTransform =
      new Pose2d(
          Units.inchesToMeters(-2.25), Units.inchesToMeters(4.625), Rotation2d.fromDegrees(0.0));

  static {
    robotToTurret =
        new Translation3d(
            Units.inchesToMeters(-2.25), // og -1.25, V2 x 4.625
            Units.inchesToMeters(4.625), // og 4, V2 y 2.25
            Units.inchesToMeters(21)); // z is LL Height, og 21
    TurretToCam = new Translation3d(Units.inchesToMeters(6.5), 0, 0);

    kFlywheelConstants.kMaxForwardOutput = 12.0;
    kFlywheelConstants.kMaxReverseOutput = -12.0;

    kFlywheelConstants.kNeutralMode = NeutralModeValue.Coast;
    kFlywheelConstants.kSupplyCurrentLimit = 40;
    kFlywheelConstants.kStatorCurrentLimit = 80;

    kFlywheelConstants.kKp = 0.0;//0.5
    kFlywheelConstants.kKs = 0.0;//0.599609375
    kFlywheelConstants.kKv = 0.0;//0.008679999969899654

    kFlywheelConstants.kEnableSupplyCurrentLimit = true;
    kFlywheelConstants.kEnableStatorCurrentLimit = true;

    kFlywheelConstants.counterClockwisePositive = true;

    TalonFXConstants followerConstants = new TalonFXConstants();
    followerConstants.id = Ports.TURRET_FLYWHEEL2;
    followerConstants.counterClockwisePositive = false;
    followerConstants.invert_sensor_phase = false;
    kFlywheelConstants.kFollowerConstants = new TalonFXConstants[] {followerConstants};

    kFlywheelConstants.kFollowerOpposeMasterDirection = false;

    // Default maps for LOB
    InterpolatingDoubleTreeMap lobHood = new InterpolatingDoubleTreeMap();
    lobHood.put(1.0, 10.0);
    lobHood.put(2.0, 12.5);
    lobHood.put(3.5, 15.0);
    lobHood.put(5.0, 18.0);

    InterpolatingDoubleTreeMap lobFly = new InterpolatingDoubleTreeMap();
    lobFly.put(1.0, 79.0);
    lobFly.put(5.0, 91.0);

    // Default maps for HUB
    InterpolatingDoubleTreeMap hubHood = new InterpolatingDoubleTreeMap();
    hubHood.put(1.0, 3.0);
    hubHood.put(2.64, 10.0);
    hubHood.put(3.5, 16.0);

    InterpolatingDoubleTreeMap hubFly =
        new InterpolatingDoubleTreeMap(); // TODO add a shit ton of values (madtown)
    hubFly.put(1.0, 35.0); // Hub
    hubFly.put(3.0, 50.0); // Depot
    hubFly.put(3.6, 52.5); // trench
    hubFly.put(4.0, 56.5); // tower side
    hubFly.put(5.0, 70.0); // Human (figure out)
    hubFly.put(5.4, 77.0); // Far-most Human again

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
      kTurretServoConstants.kRotationsPerUnitDistance = 1 / 360.0 * 65.545454; // og 104.166667

      kTurretServoConstants.kMinUnitsLimit = -390.0;
      kTurretServoConstants.kMaxUnitsLimit = 0.0;

      kTurretServoConstants.kKp = 0.0;// 4.0
      kTurretServoConstants.kKi = 0.0; //5.0
      kTurretServoConstants.kKd = 0.0;//0.1

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
      kHoodServoConstants.kMainConstants.counterClockwisePositive = true;

      kHoodServoConstants.kHomePosition = 0.0;
      kHoodServoConstants.kRotationsPerUnitDistance = 1 / 360.0 * 88.3; // og 1 / 360.0 * 96 / 1

      kHoodServoConstants.kMinUnitsLimit = 0.0;
      kHoodServoConstants.kMaxUnitsLimit = 30.0; // TODO find max

      kHoodServoConstants.kKp = 0.0;//3
      kHoodServoConstants.kKi = 0.0;//9.5
      kHoodServoConstants.kKd = 0.0;//0.02

      kHoodServoConstants.kKs = 0.0;//0.9
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
