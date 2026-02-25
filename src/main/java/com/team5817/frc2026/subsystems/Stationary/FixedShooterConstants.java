package com.team5817.frc2026.subsystems.Stationary;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team5817.frc2026.Ports;
import com.team5817.lib.drivers.Rollers.IRollerState;
import com.team5817.lib.drivers.Rollers.RollerConstantsTalonFX;
import com.team5817.lib.drivers.Rollers.RollerSubsystem.RollerControlMode;
import com.team5817.lib.drivers.Servos.ServoMotorSubsystem.TalonFXConstants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.function.DoubleSupplier;

public class FixedShooterConstants {

  /* ================= FLYWHEEL ================= */

  public static final RollerConstantsTalonFX flywheelConstants = new RollerConstantsTalonFX();

  public static final InterpolatingDoubleTreeMap HOOD_MAP_LOB;
  public static final InterpolatingDoubleTreeMap FLYWHEEL_MAP_LOB;
  public static final InterpolatingDoubleTreeMap HOOD_MAP_HUB;
  public static final InterpolatingDoubleTreeMap FLYWHEEL_MAP_HUB;

  static {
    flywheelConstants.kMaxForwardOutput = 12.0;
    flywheelConstants.kMaxReverseOutput = -12.0;
    flywheelConstants.kNeutralMode = NeutralModeValue.Coast;

    flywheelConstants.kEnableSupplyCurrentLimit = true;
    flywheelConstants.kSupplyCurrentLimit = 40;
    flywheelConstants.kEnableStatorCurrentLimit = true;
    flywheelConstants.kStatorCurrentLimit = 80;

    flywheelConstants.counterClockwisePositive = false;

    TalonFXConstants follower = new TalonFXConstants();
    follower.id = Ports.FIXED_FLYWHEEL2;
    follower.counterClockwisePositive = false;

    flywheelConstants.kFollowerConstants = new TalonFXConstants[] {follower};
    flywheelConstants.kFollowerOpposeMasterDirection = false;

    // Default maps for LOB
    InterpolatingDoubleTreeMap lobHood = new InterpolatingDoubleTreeMap();
    lobHood.put(1.0, 12.0);
    lobHood.put(2.0, 14.5);
    lobHood.put(3.5, 17.0);
    lobHood.put(5.0, 20.0);

    InterpolatingDoubleTreeMap lobFly = new InterpolatingDoubleTreeMap();
    lobFly.put(1.0, 85.0);
    lobFly.put(5.0, 95.0);

    InterpolatingDoubleTreeMap hubHood = new InterpolatingDoubleTreeMap();
    hubHood.put(1.0, 6.0);
    hubHood.put(2.0, 8.0);
    hubHood.put(3.5, 10.5);
    hubHood.put(5.0, 13.0);

    InterpolatingDoubleTreeMap hubFly = new InterpolatingDoubleTreeMap();
    hubFly.put(1.0, 55.0);
    hubFly.put(5.0, 75.0);

    HOOD_MAP_LOB = lobHood;
    FLYWHEEL_MAP_LOB = lobFly;
    HOOD_MAP_HUB = hubHood;
    FLYWHEEL_MAP_HUB = hubFly;
  }

  public enum FlywheelState implements IRollerState {
    IDLE(0.0, RollerControlMode.VOLTAGE),
    CLOSE(60.0, RollerControlMode.VELOCITY),
    FAR(80.0, RollerControlMode.VELOCITY),
    HUB(70.0, RollerControlMode.VELOCITY), // Placeholder
    LOBBING(90.0, RollerControlMode.VELOCITY); // Placeholder

    private final RollerControlMode controlMode;
    private final double toleranceRadsPerSec = 1.0;
    private DoubleSupplier supplier;

    FlywheelState(double demand, RollerControlMode controlMode) {
      this.controlMode = controlMode;
      this.supplier = () -> demand;
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

  /* ================= HOOD ================= */

  public static final class FixedShooterHoodConstants {
    public static final double kSensorToDegrees = 360.0 / 4096.0;
  }
}
