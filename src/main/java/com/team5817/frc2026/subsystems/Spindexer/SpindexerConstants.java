package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.frc2026.Ports;
import com.team5817.lib.drivers.Rollers.RollerConstantsTalonFX;
import com.team5817.lib.drivers.Servos.ServoMotorSubsystem.TalonFXConstants;

public class SpindexerConstants {

  public static final RollerConstantsTalonFX leftSpinner = new RollerConstantsTalonFX();
  public static final RollerConstantsTalonFX rightSpinner = new RollerConstantsTalonFX();

  static {
    // Left
    leftSpinner.kMainConstants.id = Ports.SPINDEXER_LEFT;
    leftSpinner.kSupplyCurrentLimit = 40;
    leftSpinner.kStatorCurrentLimit = 30;
    leftSpinner.kEnableSupplyCurrentLimit = true;
    leftSpinner.kEnableStatorCurrentLimit = true;

    TalonFXConstants followerConstants = new TalonFXConstants();
    followerConstants.id = Ports.TUNNEL_LEFT;
    followerConstants.counterClockwisePositive = false;
    followerConstants.invert_sensor_phase = false;
    leftSpinner.kFollowerConstants = new TalonFXConstants[] {followerConstants};

    leftSpinner.kFollowerOpposeMasterDirection = false;

    // Right
    rightSpinner.kMainConstants.id = Ports.SPINDEXER_RIGHT;
    rightSpinner.kSupplyCurrentLimit = 40;
    rightSpinner.kStatorCurrentLimit = 30;
    rightSpinner.kEnableSupplyCurrentLimit = true;
    rightSpinner.kEnableStatorCurrentLimit = true;
    followerConstants = new TalonFXConstants();
    followerConstants.id = Ports.TUNNEL_RIGHT;
    followerConstants.counterClockwisePositive = false;
    followerConstants.invert_sensor_phase = false;
    rightSpinner.kFollowerConstants = new TalonFXConstants[] {followerConstants};

    rightSpinner.kFollowerOpposeMasterDirection = false;
  }
}
