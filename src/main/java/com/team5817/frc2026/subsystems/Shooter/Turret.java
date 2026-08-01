package com.team5817.frc2026.subsystems.Shooter;

import com.ctre.phoenix6.hardware.CANcoder;
import com.team254.lib.geometry.Rotation2d;
import com.team5817.frc2026.Ports;
import com.team5817.frc2026.RobotVisualizer;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Turret extends StateBasedServoMotorSubsystem<Turret.State> {

  private static final double kTightError = 5.0;
  private static final double kLooseError = 6.0;

  static Supplier<Rotation2d> mRobotHeadingSupplier = () -> Rotation2d.kIdentity;
  static DoubleSupplier mTurretPositionSupplier = () -> 0.0;

  CANcoder mCanCoder;

  public Turret(
      ServoMotorIO io,
      DoubleSupplier hubAngleSupplier,
      DoubleSupplier lobAngleSupplier,
      Supplier<Rotation2d> robotHeadingSupplier) {

    super(State.STOW, io);

    mCanCoder =
        new CANcoder(Ports.TURRET_CANCODER.getDeviceNumber(), Ports.TURRET_CANCODER.getBus());

    Turret.mRobotHeadingSupplier = robotHeadingSupplier;

    State.HUB.setSupplier(hubAngleSupplier);
    State.LOBBING.setSupplier(lobAngleSupplier);

    zeroSensors(getAbsoluteTurretDegrees());

    // allow enum access to turret position
    Turret.mTurretPositionSupplier = this::getPosition;
  }

  public double getAbsoluteTurretDegrees() {
    return mCanCoder.getAbsolutePosition().getValueAsDouble()
        * (ShooterConstants.cancoderToTurretRatio)
        * 365;
  }

  public enum State implements ServoState {
    STOW(0.0, kLooseError),
    HUB(kTightError),
    LOBBING(kTightError);

    private DoubleSupplier demand;
    private final double allowableError;
    private final boolean worldOriented;

    State(double allowableError) {
      this.demand = () -> 0.0;
      this.allowableError = allowableError;
      this.worldOriented = true;
    }

    State(double supplier, double allowableError) {
      this.demand = () -> supplier;
      this.allowableError = allowableError;
      this.worldOriented = false;
    }

    void setSupplier(DoubleSupplier supplier) {
      this.demand = supplier;
    }

    @Override
    public double getDemand() {

      double target;

      if (worldOriented) {
        double demandDeg = demand.getAsDouble();
        double robotHeading = mRobotHeadingSupplier.get().getDegrees();

        // world → robot centric
        target = -demandDeg + robotHeading;
      } else {
        target = demand.getAsDouble();
      }

      // Normalize to [-180, 180)
      target = ((target + 180) % 360 + 360) % 360 - 180;

      // Shift into [-300, 0] range
      if (target > 0) {
        target -= 360;
      }

      return target;
    }

    @Override
    public double getAllowableError() {
      return allowableError;
    }

    @Override
    public ControlState getControlState() {
      return ControlState.POSITION_VOLTAGE;
    }

    @Override
    public boolean isDisabled() {
      return false;
    }
  }

  @Override
  public void outputTelemetry() {
    RobotVisualizer.updateTurretPose(getPosition());
    super.outputTelemetry();
  }
}
