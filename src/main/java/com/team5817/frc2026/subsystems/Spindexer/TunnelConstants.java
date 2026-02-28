package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.frc2026.Ports;
import com.team5817.lib.drivers.Rollers.RollerConstantsTalonFX;
import com.team5817.lib.drivers.Servos.ServoMotorSubsystem.TalonFXConstants;

public class TunnelConstants {

  public static final RollerConstantsTalonFX leftRoller = new RollerConstantsTalonFX();
  public static final RollerConstantsTalonFX rightRoller = new RollerConstantsTalonFX();

  static {
    // Left
    leftRoller.kMainConstants.id = Ports.SPINDEXER_2;
    leftRoller.kSupplyCurrentLimit = 40;
    leftRoller.kStatorCurrentLimit = 30;
    leftRoller.kEnableSupplyCurrentLimit = true;
    leftRoller.kEnableStatorCurrentLimit = true;
    TalonFXConstants followerConstants = new TalonFXConstants();
    followerConstants.id = Ports.TUNNEL_LEFT;
    followerConstants.counterClockwisePositive = false;
    followerConstants.invert_sensor_phase = false;
    leftRoller.kFollowerConstants = new TalonFXConstants[] {followerConstants};

    leftRoller.kFollowerOpposeMasterDirection = false;

    // Right
    rightRoller.kMainConstants.id = Ports.SPINDEXER_1;
    rightRoller.kSupplyCurrentLimit = 40;
    rightRoller.kStatorCurrentLimit = 30;
    rightRoller.kEnableSupplyCurrentLimit = true;
    rightRoller.kEnableStatorCurrentLimit = true;
    followerConstants = new TalonFXConstants();
    followerConstants.id = Ports.TUNNEL_RIGHT;
    followerConstants.counterClockwisePositive = false;
    followerConstants.invert_sensor_phase = false;
    rightRoller.kFollowerConstants = new TalonFXConstants[] {followerConstants};

    rightRoller.kFollowerOpposeMasterDirection = false;
  }
}
