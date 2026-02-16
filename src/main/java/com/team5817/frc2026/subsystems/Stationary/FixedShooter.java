package com.team5817.frc2026.subsystems.Stationary;

import com.team5817.frc2026.RobotVisualizer;
import com.team5817.frc2026.planners.ShootingPlanner;
import com.team5817.frc2026.planners.ShootingTarget;
import com.team5817.frc2026.subsystems.Stationary.FixedShooterConstants.FlywheelState;
import com.team5817.lib.drivers.Rollers.RollerSubsystem;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.Request;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.Logger;

public class FixedShooter extends Subsystem {

  private final ShootingPlanner planner;   
  @Getter private final FixedShooterHood fixedHood;
  @Getter private final RollerSubsystem<FlywheelState> fixedFlywheel;
  
  
  public FixedShooter(
      ServoMotorIO hoodIO,
      RollerSubsystemIO flywheelIO,
      ShootingPlanner shootingPlanner) {

    this.planner = shootingPlanner;
    fixedHood = new FixedShooterHood(hoodIO);
    fixedFlywheel = new RollerSubsystem<>(FlywheelState.IDLE, "FixedShooter/Flywheel", flywheelIO);
  }

  @Getter
  @Accessors(prefix = "m")
  private State mState = State.IDLE;

  @Getter @Setter private State desiredState = State.IDLE;
  private boolean atState = false;

  public enum State {
    IDLE(FixedShooterHood.State.STOW, FlywheelState.IDLE),
    CLOSE(FixedShooterHood.State.CLOSE, FlywheelState.CLOSE),
    FAR(FixedShooterHood.State.FAR, FlywheelState.FAR),
    HUB(FixedShooterHood.State.FAR, FlywheelState.HUB),
    LOBBING(FixedShooterHood.State.FAR, FlywheelState.LOBBING);

    final FixedShooterHood.State hoodState;
    final FlywheelState flywheelState;

    State(
        FixedShooterHood.State hoodState,
        FlywheelState flywheelState) {
      this.hoodState = hoodState;
      this.flywheelState = flywheelState;
    }
  }

  @Override
  public void periodic() {
  switch (desiredState) {

    case HUB:
      fixedHood.setDynamicSupplier(
          planner.getHoodAngleSupplier(ShootingTarget.HUB));

      FlywheelState.HUB.setSupplier(
          planner.getFlywheelSpeedSupplier(ShootingTarget.HUB));
      break;

    case LOBBING:
      fixedHood.setDynamicSupplier(
          planner.getHoodAngleSupplier(ShootingTarget.LOB));

      FlywheelState.LOBBING.setSupplier(
          planner.getFlywheelSpeedSupplier(ShootingTarget.LOB));
      break;

    default:
      break;
  }

  atState = fixedHood.atState() && fixedFlywheel.atState();

  if (mState != desiredState) {
    fixedHood.setState(desiredState.hoodState);
    fixedFlywheel.setState(desiredState.flywheelState);
    if (atState) {
      mState = desiredState;
    }
  }
}

  public Request stateRequest(State state) {
    return new Request() {
      @Override
      public void act() {
        setDesiredState(state);
      }

      @Override
      public boolean isFinished() {
        return atState;
      }
    };
  }

  @Override
  public void readPeriodicInputs() {
    fixedHood.readPeriodicInputs();
    fixedFlywheel.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    fixedHood.writePeriodicOutputs();
    fixedFlywheel.writePeriodicOutputs();
  }

  @Override
  public void outputTelemetry() {
    Logger.recordOutput("FixedShooter/CurrentState", mState);
    Logger.recordOutput("FixedShooter/DesiredState", desiredState);
    fixedHood.outputTelemetry();
    fixedFlywheel.outputTelemetry();
    RobotVisualizer.updateFixedFlywheel(fixedFlywheel.getVelocity());
    RobotVisualizer.updateFixedHood(fixedHood.getPosition());
    super.outputTelemetry();
  }
}
