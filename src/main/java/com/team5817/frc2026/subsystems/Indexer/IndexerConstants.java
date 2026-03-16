package com.team5817.frc2026.subsystems.Indexer;

import com.team5817.frc2026.Ports;
import com.team5817.lib.drivers.Rollers.RollerConstantsTalonFX;

public class IndexerConstants {

  public static final RollerConstantsTalonFX kSpindexerConstants = new RollerConstantsTalonFX();
  public static final RollerConstantsTalonFX kTunnelConstants = new RollerConstantsTalonFX();

  static {

    kSpindexerConstants.kMainConstants.id = Ports.SPINDEXER;
    kSpindexerConstants.kSupplyCurrentLimit = 40;
    kSpindexerConstants.kStatorCurrentLimit = 30;
    kSpindexerConstants.kEnableSupplyCurrentLimit = true;
    kSpindexerConstants.kEnableStatorCurrentLimit = true;

    kTunnelConstants.kMainConstants.id = Ports.TUNNEL;
    kTunnelConstants.kSupplyCurrentLimit = 40;
    kTunnelConstants.kStatorCurrentLimit = 30;
    kTunnelConstants.kEnableSupplyCurrentLimit = true;
    kTunnelConstants.kEnableStatorCurrentLimit = true;
    kTunnelConstants.kMainConstants.counterClockwisePositive = true;

  }
}
