package com.team5817.frc2025.subsystems.Shooter;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2025.Ports;
import com.team5817.lib.drivers.Rollers.IRollerState;
import com.team5817.lib.drivers.Rollers.RollerConstantsTalonFX;
import com.team5817.lib.drivers.Rollers.RollerSubsystem.RollerControlMode;
import com.team5817.lib.drivers.Servos.ServoConstants;
import com.team5817.lib.drivers.Servos.ServoMotorSubsystem.TalonFXConstants;

import lombok.Getter;

public class ShooterConstants {
    
    public static final RollerConstantsTalonFX flywheelConstants = new RollerConstantsTalonFX();

    static{
       
        flywheelConstants.kMaxForwardOutput = 12.0;
        flywheelConstants.kMaxReverseOutput = -12.0;

        flywheelConstants.kNeutralMode = NeutralModeValue.Coast;
        flywheelConstants.kSupplyCurrentLimit = 40;
        flywheelConstants.kStatorCurrentLimit = 80;

        flywheelConstants.kEnableSupplyCurrentLimit = true;
        flywheelConstants.kEnableStatorCurrentLimit = true;

        flywheelConstants.counterClockwisePositive = false;

        //Follower Motor (internally ran in robot container when getting my lead flywheel)

        TalonFXConstants followerConstants = new TalonFXConstants();
        followerConstants.id = Ports.FLYWHEEL_2;
        followerConstants.counterClockwisePositive = false;
        followerConstants.invert_sensor_phase = false;
        flywheelConstants.kFollowerConstants = new TalonFXConstants [] {followerConstants};


        flywheelConstants.kFollowerOpposeMasterDirection = false;
    }

public enum FlywheelState implements IRollerState{

    IDLE(0.0),
    SHOOT(12),
    EJECT(3);

    @Getter
    private final double demand;

    @Getter
    private final RollerControlMode controlMode = RollerControlMode.VOLTAGE;

    FlywheelState(double demand) {
      this.demand = demand;
    }

}

public static final class TurretConstants{
    public static final ServoConstants kTurretServoConstants = new ServoConstants();

    static{
        kTurretServoConstants.kName="Turret ";

        kTurretServoConstants.kMainConstants.id = Ports.TURRET;
        kTurretServoConstants.kMainConstants.counterClockwisePositive = true;

        kTurretServoConstants.kHomePosition = 0.0;
        kTurretServoConstants.kRotationsPerUnitDistance = 1/360.0; 

        //Soft limits, lets adjust later throughout cad development
        kTurretServoConstants.kMinUnitsLimit = -180.0;
        kTurretServoConstants.kMaxUnitsLimit = 180.0;

        //PID (this is just a placeholder for now, )
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
            

        // homing (safe defaults?)
        kTurretServoConstants.kHomingTimeout = 0.5;
        kTurretServoConstants.kHomingOutput = -0.25;
        kTurretServoConstants.kHomingVelocityWindow = 1.0;
    }
  }

    public static final class HoodConstants{
        public static final ServoConstants kHoodServoConstants = new ServoConstants();
        

        static{
            kHoodServoConstants.kName="Hood";

            kHoodServoConstants.kMainConstants.id = Ports.HOOD;
            kHoodServoConstants.kMainConstants.counterClockwisePositive = true;

            kHoodServoConstants.kHomePosition = 0.0;
            kHoodServoConstants.kRotationsPerUnitDistance = 1/360.0;

            //Soft limits, ill adjust later throughout cad development
            kHoodServoConstants.kMinUnitsLimit = -180.0;
            kHoodServoConstants.kMaxUnitsLimit = 180.0;

            //PID copied from last year (this is just a placeholder for now)
            kHoodServoConstants.kKp = 1.5;
            kHoodServoConstants.kKi = 0.0;
            kHoodServoConstants.kKd = 0.0;

            kHoodServoConstants.kKs = 0.0;
            kHoodServoConstants.kKv = 0.0;
            kHoodServoConstants.kKa = 0.0;
            kHoodServoConstants.kKg = 0.0;

            kHoodServoConstants.kGravityType =
                GravityTypeValue.Arm_Cosine;

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


}