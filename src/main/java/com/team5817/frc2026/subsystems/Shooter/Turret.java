package com.team5817.frc2026.subsystems.Shooter;

import com.team254.lib.geometry.Rotation2d;
import com.team5817.frc2026.RobotVisualizer;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoState;
import com.team5817.lib.drivers.Servos.StateBasedServoMotorSubsystem;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Turret extends StateBasedServoMotorSubsystem<Turret.State> {

  private static final double kTightError = 3.0;
  private static final double kLooseError = 4.0;

  static Supplier<Rotation2d> mRobotHeadingSupplier = () -> Rotation2d.kIdentity;

  public Turret(
      ServoMotorIO io,
      DoubleSupplier hubAngleSupplier,
      DoubleSupplier hubVelocityFFSupplier,
      DoubleSupplier lobAngleSupplier,
      DoubleSupplier lobVelocityFFSupplier,
      Supplier<Rotation2d> robotHeadingSupplier) {
    super(State.STOW, io);
    Turret.mRobotHeadingSupplier = robotHeadingSupplier;

    State.HUB.setDemandSupplier(hubAngleSupplier);
    State.HUB.setFFSupplier(hubVelocityFFSupplier);
    State.LOBBING.setDemandSupplier(lobAngleSupplier);
    State.LOBBING.setFFSupplier(lobVelocityFFSupplier);
  }

  public enum State implements ServoState {
    HEADINGTEST(kTightError),
    STOW(0.0, kLooseError),
    HUB(kTightError), // demand and ff suppliers set in constructor
    LOBBING(kTightError); // demand and ff suppliers set in constructor

    private DoubleSupplier demandSupplier;
    private DoubleSupplier ffSupplier;
    @Getter private final double allowableError;
    private final boolean worldOriented;

    State(double allowableError) {
      this.demandSupplier = () -> 0.0;
      this.ffSupplier = () -> 0.0;
      this.allowableError = allowableError;
      this.worldOriented = true;
    }

    State(double fixedDemand, double allowableError) {
      this.demandSupplier = () -> fixedDemand;
      this.ffSupplier = () -> 0.0;
      this.allowableError = allowableError;
      this.worldOriented = false;
    }

    public void setDemandSupplier(DoubleSupplier supplier) {
      if (supplier != null) this.demandSupplier = supplier;
    }

    public void setFFSupplier(DoubleSupplier supplier) {
      if (supplier != null) this.ffSupplier = supplier;
    }

    @Override
    public double getDemand() {
      if (worldOriented) {
        double worldOrientedDemand =
            demandSupplier.getAsDouble() - mRobotHeadingSupplier.get().getDegrees();
        if (worldOrientedDemand > 180) return worldOrientedDemand - 360;
        else if (worldOrientedDemand < -180) return worldOrientedDemand + 360;
        return worldOrientedDemand;
      }
      return demandSupplier.getAsDouble();
    }
    @Override
    public double getVelocityFF() {
      return ffSupplier.getAsDouble();
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

    double demand = getState().getDemand();
    double position = getPosition();
    boolean atState = atState();
    double diff = Math.abs(position - demand);

    Logger.recordOutput("Turret/Position", position);
    Logger.recordOutput("Turret/Demand", demand);
    Logger.recordOutput("Turret/AtStateCheck", atState);
    Logger.recordOutput("Turret/AtStateDiff", diff);

    super.outputTelemetry();
  }
}
