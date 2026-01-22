package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.frc2026.Ports;
import com.team5817.lib.drivers.Rollers.IRollerState;
import com.team5817.lib.drivers.Rollers.RollerConstantsTalonFX;
import com.team5817.lib.drivers.Rollers.RollerSubsystem.RollerControlMode;

public class SpindexerConstants {

  public static final RollerConstantsTalonFX leftRoller = new RollerConstantsTalonFX();
  public static final RollerConstantsTalonFX rightRoller = new RollerConstantsTalonFX();

  static {
    // Left
    leftRoller.kMainConstants.id = Ports.SPINDEXER_1;
    leftRoller.kSupplyCurrentLimit = 40;
    leftRoller.kStatorCurrentLimit = 80;
    leftRoller.kEnableSupplyCurrentLimit = true;
    leftRoller.kEnableStatorCurrentLimit = true;

    // Right
    rightRoller.kMainConstants.id = Ports.SPINDEXER_2;
    rightRoller.kSupplyCurrentLimit = 40;
    rightRoller.kStatorCurrentLimit = 80;
    rightRoller.kEnableSupplyCurrentLimit = true;
    rightRoller.kEnableStatorCurrentLimit = true;
  }

  public enum SpindexerState implements IRollerState {
    IDLE(0),
    CLOCKWISE(10),
    COUNTERCLOCKWISE(-10),
    EXHAUST(-12);

    private final double demand;

    SpindexerState(double demand) {
      this.demand = demand;
    }

    @Override
    public double getDemand() {
      return demand;
    }

    @Override
    public RollerControlMode getControlMode() {
      return RollerControlMode.VOLTAGE;
    }

    @Override
    public double getToleranceRadsPerSec() {
      return 0.0;
    }
  }
}
