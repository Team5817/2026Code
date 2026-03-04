package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.frc2026.Ports;
import com.team5817.lib.drivers.Rollers.RollerConstantsTalonFX;

public class TunnelConstants {

  public static final RollerConstantsTalonFX TUNNEL =
      new RollerConstantsTalonFX();

  static {
    TUNNEL.kMainConstants.id = Ports.TUNNEL;

    TUNNEL.kSupplyCurrentLimit = 40;
    TUNNEL.kStatorCurrentLimit = 30;
    TUNNEL.kEnableSupplyCurrentLimit = true;
    TUNNEL.kEnableStatorCurrentLimit = true;

    TUNNEL.kMainConstants.counterClockwisePositive = true;
  }
}