package com.team5817.frc2026.subsystems.Spindexer;

import com.team5817.frc2026.Ports;
import com.team5817.lib.drivers.Rollers.IRollerState;
import com.team5817.lib.drivers.Rollers.RollerConstantsTalonFX;
import com.team5817.lib.drivers.Rollers.RollerSubsystem.RollerControlMode;
import com.team5817.lib.drivers.Servos.ServoMotorSubsystem.TalonFXConstants;

public class SpindexerConstants {

  /** Spindexer Constants */
  public static final RollerConstantsTalonFX spindexerConstants = new RollerConstantsTalonFX();

  static {
    spindexerConstants.kMainConstants.id = Ports.SPINDEXER_1;
    spindexerConstants.kMainConstants.counterClockwisePositive = true;

    spindexerConstants.kSupplyCurrentLimit = 40;
    spindexerConstants.kStatorCurrentLimit = 80;
    spindexerConstants.kEnableSupplyCurrentLimit = true;
    spindexerConstants.kEnableStatorCurrentLimit = true;
    spindexerConstants.kMaxForwardOutput = 12.0;
    spindexerConstants.kMaxReverseOutput = -12.0;

    // Follower
    TalonFXConstants follower = new TalonFXConstants();
    follower.id = Ports.SPINDEXER_2;
    follower.counterClockwisePositive = false;

    spindexerConstants.kFollowerConstants = new TalonFXConstants[] {follower};
    spindexerConstants.kFollowerOpposeMasterDirection = false;
  }

  public static enum SpindexerState implements IRollerState {
    IDLE(0.0, RollerControlMode.VOLTAGE),
    INTAKING(-10.0, RollerControlMode.VOLTAGE),
    EXHAUST(6.0, RollerControlMode.VOLTAGE);

    private final double demand;
    private final RollerControlMode controlMode;

    SpindexerState(double demand, RollerControlMode controlMode) {
      this.demand = demand;
      this.controlMode = controlMode;
    }

    @Override
    public double getDemand() {
      return demand;
    }

    @Override
    public RollerControlMode getControlMode() {
      return controlMode;
    }

    @Override
    public double getToleranceRadsPerSec() {
      return 0.0;
    }
  }
}
