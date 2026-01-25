package com.team5817.frc2026.subsystems.Shooter;

import com.team254.lib.geometry.Rotation2d;
import com.team5817.frc2026.RobotVisualizer;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Turret extends StateBasedServoMotorSubsystem<Turret.State> {

  private static final double kTightError = 1.0;
  private static final double kLooseError = 4.0;

  static Supplier<Rotation2d> mRobotHeadingSupplier = () -> Rotation2d.kIdentity;

  public Turret(
      ServoMotorIO io,
      DoubleSupplier hubAngleSupplier,
      DoubleSupplier lobAngleSupplier,
      Supplier<Rotation2d> robotHeadingSupplier) {
    super(State.STOW, io);
    Turret.mRobotHeadingSupplier = robotHeadingSupplier;
    State.HUB.setSupplier(hubAngleSupplier);
    State.LOBBING.setSupplier(lobAngleSupplier);
  }

  public enum State implements ServoState {
    HEADINGTEST(kTightError),
    STOW(0.0, kLooseError),
    HUB(kTightError), // set in constructor
    LOBBING(kTightError); // set in constructor

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
      if (worldOriented){ 
        double worldOrientedDemand = demand.getAsDouble() - mRobotHeadingSupplier.get().getDegrees();
        if (worldOrientedDemand > 180) return worldOrientedDemand -360;
        else if (worldOrientedDemand < -180) return worldOrientedDemand +360;
          //return worldOrientedDemand -(180 * Math.signum(worldOrientedDemand));
        return worldOrientedDemand;
      }
      return demand.getAsDouble();
    }

    @Override
    public double getAllowableError() {
      return allowableError;
    }

    @Override
    public ControlState getControlState() {
      return ControlState.POSITION;
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
