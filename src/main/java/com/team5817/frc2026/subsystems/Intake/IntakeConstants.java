package com.team5817.frc2026.subsystems.Intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2026.Ports;
import com.team5817.lib.drivers.Rollers.RollerConstantsTalonFX;
import com.team5817.lib.drivers.Servos.ServoConstants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class IntakeConstants {
  public static final class DeployConstants {

    public static final ServoConstants kRackLeftServoConstants = new ServoConstants();
    public static final ServoConstants kRackRightServoConstants = new ServoConstants();

    static {
      kRackLeftServoConstants.kName = "Intake/Rack/Left";

      kRackLeftServoConstants.kMainConstants.id = Ports.RACK_LEFT;
      kRackLeftServoConstants.kMainConstants.counterClockwisePositive = false;

      kRackLeftServoConstants.kHomePosition = 0.3175;
      kRackLeftServoConstants.kRotationsPerUnitDistance = 143.6 * (3.28125 / 11.458);
      kRackLeftServoConstants.kMaxUnitsLimit = 0.317;
      kRackLeftServoConstants.kMinUnitsLimit = 0.0;

      kRackLeftServoConstants.kKp = 1.5;
      kRackLeftServoConstants.kKi = 0.0;
      kRackLeftServoConstants.kKd = 0.0;
      kRackLeftServoConstants.kKa = 0;
      kRackLeftServoConstants.kKs = 0;
      kRackLeftServoConstants.kKv = 0;
      kRackLeftServoConstants.kKg = 0;

      kRackLeftServoConstants.kCruiseVelocity = 1;
      kRackLeftServoConstants.kAcceleration = 1000000000;

      kRackLeftServoConstants.kMaxForwardOutput = 12.0;
      kRackLeftServoConstants.kMaxReverseOutput = -12.0;

      kRackLeftServoConstants.kEnableSupplyCurrentLimit = true;
      kRackLeftServoConstants.kSupplyCurrentLimit = 80;
      kRackLeftServoConstants.kEnableStatorCurrentLimit = true;
      kRackLeftServoConstants.kStatorCurrentLimit = 60;

      kRackLeftServoConstants.kNeutralMode = NeutralModeValue.Coast;
      kRackLeftServoConstants.kHomingOutput = 0.3;
      kRackLeftServoConstants.kHomingTimeout = 1.0;
      kRackLeftServoConstants.kHomingVelocityWindow = .05;

      kRackRightServoConstants.kName = "Intake/Rack/Right";

      kRackRightServoConstants.kMainConstants.id = Ports.RACK_RIGHT;
      kRackRightServoConstants.kMainConstants.counterClockwisePositive = true;

      kRackRightServoConstants.kHomePosition = 0.3175;
      kRackRightServoConstants.kRotationsPerUnitDistance = 143.6 * (3.28125 / 11.458);
      kRackRightServoConstants.kMaxUnitsLimit = 0.317;
      kRackRightServoConstants.kMinUnitsLimit = 0.0;

      kRackRightServoConstants.kKp = 1.5;
      kRackRightServoConstants.kKi = 0.0;
      kRackRightServoConstants.kKd = 0.0;
      kRackRightServoConstants.kKa = 0;
      kRackRightServoConstants.kKs = 0;
      kRackRightServoConstants.kKv = 0;
      kRackRightServoConstants.kKg = 0;

      kRackRightServoConstants.kCruiseVelocity = 1;
      kRackRightServoConstants.kAcceleration = 1000000000;

      kRackRightServoConstants.kMaxForwardOutput = 12.0;
      kRackRightServoConstants.kMaxReverseOutput = -12.0;

      kRackRightServoConstants.kEnableSupplyCurrentLimit = true;
      kRackRightServoConstants.kSupplyCurrentLimit = 80;
      kRackRightServoConstants.kEnableStatorCurrentLimit = true;
      kRackRightServoConstants.kStatorCurrentLimit = 60;

      kRackRightServoConstants.kNeutralMode = NeutralModeValue.Coast;
      kRackRightServoConstants.kHomingOutput = 0.3;
      kRackRightServoConstants.kHomingTimeout = 1.0;
      kRackRightServoConstants.kHomingVelocityWindow = .05;
    }
  }

  public static final class RollerConstants {
    public static final InterpolatingDoubleTreeMap voltageMap = new InterpolatingDoubleTreeMap();

    static {
      voltageMap.put(0.0, -4.0);
      voltageMap.put(5.0, -12.0);
    }

    public static RollerConstantsTalonFX kMotorConstants = new RollerConstantsTalonFX();

    static {
      kMotorConstants.kSupplyCurrentLimit = 40;
      kMotorConstants.kStatorCurrentLimit = 80;
      kMotorConstants.kEnableSupplyCurrentLimit = true;
      kMotorConstants.kEnableStatorCurrentLimit = true;
      kMotorConstants.kMaxForwardOutput = 12.0;
      kMotorConstants.kMaxReverseOutput = -12.0;
    }
  }
}
