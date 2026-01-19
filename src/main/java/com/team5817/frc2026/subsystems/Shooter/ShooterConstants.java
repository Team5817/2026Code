package com.team5817.frc2026.subsystems.Shooter;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2026.Ports;
import com.team5817.lib.drivers.Rollers.RollerConstantsTalonFX;
import com.team5817.lib.drivers.Rollers.IRollerState;
import com.team5817.lib.drivers.Rollers.RollerSubsystem.RollerControlMode;
import java.util.function.DoubleSupplier;
import com.team5817.lib.drivers.Servos.ServoConstants;
import com.team5817.lib.drivers.Servos.ServoMotorSubsystem.TalonFXConstants;

/**
 * Centralized shooter constants and per-target lookup tables.
 *
 * This cleaned version consolidates map initialization into a single
 * static initializer to avoid duplicate/ malformed blocks introduced
 * during earlier edits.
 */
public class ShooterConstants {

    public static final RollerConstantsTalonFX flywheelConstants = new RollerConstantsTalonFX();

    // Also expose per-target maps as named constants so enums can reference them in their constructors.
    public static final edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap HOOD_MAP_LOB;
    public static final edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap FLYWHEEL_MAP_LOB;
    public static final edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap HOOD_MAP_HUB;
    public static final edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap FLYWHEEL_MAP_HUB;

    static {
        // Basic flywheel/talon defaults
        flywheelConstants.kMaxForwardOutput = 12.0;
        flywheelConstants.kMaxReverseOutput = -12.0;

        flywheelConstants.kNeutralMode = NeutralModeValue.Coast;
        flywheelConstants.kSupplyCurrentLimit = 40;
        flywheelConstants.kStatorCurrentLimit = 80;

        flywheelConstants.kEnableSupplyCurrentLimit = true;
        flywheelConstants.kEnableStatorCurrentLimit = true;

        flywheelConstants.counterClockwisePositive = false;

        // Follower Motor
        TalonFXConstants followerConstants = new TalonFXConstants();
        followerConstants.id = Ports.FLYWHEEL_2;
        followerConstants.counterClockwisePositive = false;
        followerConstants.invert_sensor_phase = false;
        flywheelConstants.kFollowerConstants = new TalonFXConstants[] { followerConstants };

        flywheelConstants.kFollowerOpposeMasterDirection = false;

        // Default maps for LOB
        edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap lobHood = new edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap();
        lobHood.put(1.0, 10.0);
        lobHood.put(2.0, 12.5);
        lobHood.put(3.5, 15.0);
        lobHood.put(5.0, 18.0);

        edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap lobFly = new edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap();
        lobFly.put(1.0, 1000.0);
        lobFly.put(2.0, 2000.0);
        lobFly.put(3.5, 3000.0);
        lobFly.put(5.0, 4000.0);

        // Default maps for HUB
        edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap hubHood = new edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap();
        hubHood.put(1.0, 5.0);
        hubHood.put(2.0, 7.5);
        hubHood.put(3.5, 10.0);
        hubHood.put(5.0, 12.0);

        edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap hubFly = new edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap();
        hubFly.put(1.0, 1500.0);
        hubFly.put(2.0, 2500.0);
        hubFly.put(3.5, 3500.0);
        hubFly.put(5.0, 4500.0);

        // Publish named constants (used by ShootingTarget constructor if desired)
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
            kTurretServoConstants.kRotationsPerUnitDistance = 1 / 360.0;

            // Soft limits
            kTurretServoConstants.kMinUnitsLimit = -180.0;
            kTurretServoConstants.kMaxUnitsLimit = 180.0;

            // PID (placeholder)
            kTurretServoConstants.kKp = 2.0;
            kTurretServoConstants.kKi = 0.0;
            kTurretServoConstants.kKd = 0.0;

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
            kTurretServoConstants.kStatorCurrentLimit = 10;

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
            kHoodServoConstants.kMainConstants.counterClockwisePositive = true;

            kHoodServoConstants.kHomePosition = 0.0;
            kHoodServoConstants.kRotationsPerUnitDistance = 1 / 360.0;

            // Soft limits
            kHoodServoConstants.kMinUnitsLimit = -180.0;
            kHoodServoConstants.kMaxUnitsLimit = 180.0;

            // PID (placeholder)
            kHoodServoConstants.kKp = 1.5;
            kHoodServoConstants.kKi = 0.0;
            kHoodServoConstants.kKd = 0.0;

            kHoodServoConstants.kKs = 0.0;
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

        /**
     * Flywheel states for the Shooter roller subsystem.
     *
     * This enum implements IRollerState so it can be used with RollerSubsystem.
     * It supports setting a dynamic DoubleSupplier for velocity-demand states (HUB/LOB)
     * so the ShootingPlanner can provide live setpoints.
     */
    public enum FlywheelState implements IRollerState {
        IDLE(0.0, RollerControlMode.VOLTAGE),
        CLOSE(0.0, RollerControlMode.VELOCITY),
        FAR(0.0, RollerControlMode.VELOCITY),
        HUB(0.0, RollerControlMode.VELOCITY),
        LOB(0.0, RollerControlMode.VELOCITY);

        private final RollerControlMode controlMode;
        private final double toleranceRadsPerSec = 0.1;
        private DoubleSupplier supplier = () -> 0.0;

        FlywheelState(double staticDemand, RollerControlMode controlMode) {
            this.controlMode = controlMode;
            this.supplier = () -> staticDemand;
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