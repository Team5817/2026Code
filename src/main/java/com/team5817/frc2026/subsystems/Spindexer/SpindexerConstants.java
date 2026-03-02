package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.frc2026.Ports;
import com.team5817.lib.drivers.Rollers.RollerConstantsTalonFX;
import com.team5817.lib.drivers.Servos.ServoMotorSubsystem.TalonFXConstants;

public class SpindexerConstants {

  public static final RollerConstantsTalonFX Spinner2 = new RollerConstantsTalonFX();
  public static final RollerConstantsTalonFX Spinner1 = new RollerConstantsTalonFX();

  static {
    // Left
    Spinner2.kMainConstants.id = Ports.SPINDEXER_2;
    Spinner2.kSupplyCurrentLimit = 40;
    Spinner2.kStatorCurrentLimit = 30;
    Spinner2.kEnableSupplyCurrentLimit = true;
    Spinner2.kEnableStatorCurrentLimit = true;

    TalonFXConstants followerConstants = new TalonFXConstants();
    followerConstants.id = Ports.TUNNEL_SECONDARY;
    followerConstants.counterClockwisePositive = true;
    followerConstants.invert_sensor_phase = false;
    Spinner2.kFollowerConstants = new TalonFXConstants[] {followerConstants};

    Spinner2.kFollowerOpposeMasterDirection = false;

    // Right
    Spinner1.kMainConstants.id = Ports.SPINDEXER_1;
    Spinner1.kSupplyCurrentLimit = 40;
    Spinner1.kStatorCurrentLimit = 30;
    Spinner1.kEnableSupplyCurrentLimit = true;
    Spinner1.kEnableStatorCurrentLimit = true;
    followerConstants = new TalonFXConstants();
    followerConstants.id = Ports.TUNNEL_PRIMARY;
    followerConstants.counterClockwisePositive = false;
    followerConstants.invert_sensor_phase = false;
    Spinner1.kFollowerConstants = new TalonFXConstants[] {followerConstants};

    Spinner1.kFollowerOpposeMasterDirection = false;
  }
}
