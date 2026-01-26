package com.team5817.frc2026.subsystems.Shooter;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team5817.frc2026.RobotVisualizer;
import com.team5817.frc2026.planners.ShootingPlanner;
import com.team5817.frc2026.planners.ShootingPlannerI;
import com.team5817.frc2026.planners.ShootingTarget;
import com.team5817.frc2026.subsystems.Shooter.ShooterConstants.FlywheelState;
import com.team5817.lib.drivers.Rollers.RollerSubsystem;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.Request;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.Logger;

public class Shooter extends Subsystem {

  @Getter private final Turret turret;
  @Getter private final Hood hood;
  @Getter private final RollerSubsystem<ShooterConstants.FlywheelState> flywheel;

  @Getter private ShootingPlannerI planner;

  public Shooter(
      ServoMotorIO turretIO,
      ServoMotorIO hoodIO,
      RollerSubsystemIO flywheelIO,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> robotVelocitySupplier,
      DoubleSupplier timeSinceVision) {
    Supplier<Rotation2d> robotHeadingSupplier = () -> robotPoseSupplier.get().getRotation();
    Supplier<Pose2d> shooterPoseSupplier =
        () -> robotPoseSupplier.get().transformBy(ShooterConstants.shooterTransform);
    this.planner =
        new ShootingPlanner(
            shooterPoseSupplier, robotVelocitySupplier, this::isAtState, timeSinceVision);
    this.turret =
        new Turret(
            turretIO,
            planner.getTurretAngleSupplier(ShootingTarget.HUB),
            planner.getTurretAngleSupplier(ShootingTarget.LOB),
            robotHeadingSupplier);
    this.hood =
        new Hood(
            hoodIO,
            planner.getHoodAngleSupplier(ShootingTarget.HUB),
            planner.getHoodAngleSupplier(ShootingTarget.LOB));
    FlywheelState.HUB.setSupplier(planner.getFlywheelSpeedSupplier(ShootingTarget.HUB));
    FlywheelState.LOBBING.setSupplier(planner.getFlywheelSpeedSupplier(ShootingTarget.LOB));
    this.flywheel =
        new RollerSubsystem<ShooterConstants.FlywheelState>(
            FlywheelState.IDLE, "Shooter/Flywheel", flywheelIO);

  // Provide planner with a live supplier that checks the actual component states.
  // This avoids stale/lagging values when the planner is asked whether we are at state
  // before the Shooter's internal `atState` field has been updated by periodic().
  ((ShootingPlanner) this.planner)
    .setAtStateSupplier(() -> turret.atState() && hood.atState() && flywheel.atState() && !forcedStow && mState != State.STOW);
  }

  @Getter
  @Accessors(prefix = "m")
  private State mState = State.STOW;

  @Getter @Setter private State desiredState = State.HUB;
  public boolean isAtState() {
    return atState;
  }
  private boolean atState = false;
  private boolean forcedStow = false;

  public enum State {
    STOW(Turret.State.STOW, Hood.State.STOW, ShooterConstants.FlywheelState.IDLE),
    CLOSE(Turret.State.STOW, Hood.State.CLOSE, ShooterConstants.FlywheelState.CLOSE),
    FAR(Turret.State.STOW, Hood.State.FAR, ShooterConstants.FlywheelState.FAR),
    HUB(Turret.State.HUB, Hood.State.HUB, ShooterConstants.FlywheelState.HUB),
    LOB(Turret.State.LOBBING, Hood.State.LOBBING, ShooterConstants.FlywheelState.LOBBING);

    final Turret.State turretState;
    final Hood.State hoodState;
    final ShooterConstants.FlywheelState flywheelState;

    State(
        Turret.State turretState,
        Hood.State hoodState,
        ShooterConstants.FlywheelState flywheelState) {
      this.turretState = turretState;
      this.hoodState = hoodState;
      this.flywheelState = flywheelState;
    }
  }

  @Override
  public void periodic() {
    if (followPlan) setDesiredState(planner.recommendedShooterState());
    if (forcedStow) {
      desiredState = State.STOW;
    }
    
    atState = turret.atState() && hood.atState() && flywheel.atState() && !forcedStow;
    
    Logger.recordOutput(
    "Shooter/AtStateDetails",
    "Turret: " + turret.atState()
    + ", Hood: " + hood.atState()
    + ", Flywheel: " + flywheel.atState()
    + ", ForcedStow: " + forcedStow
    + ", Not Stowing: " + (mState != State.STOW));

    if (mState != desiredState) {
      turret.setState(desiredState.turretState);
      hood.setState(desiredState.hoodState);
      flywheel.setState(desiredState.flywheelState);
      if (atState) {
        mState = desiredState;
      }
    }
  }

  @Override
  public void outputTelemetry() {
    Logger.recordOutput("Shooter/Current State", mState);
    Logger.recordOutput("Shooter/Desired State", desiredState);
    Logger.recordOutput("Shooter/Forced Stow", forcedStow);
    Logger.recordOutput("Shooter/Follow Plan", followPlan);
    turret.outputTelemetry();
    hood.outputTelemetry();
    flywheel.outputTelemetry();
    RobotVisualizer.updateFlyWheel(flywheel.getVelocity());
    super.outputTelemetry();
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
    flywheel.readPeriodicInputs();
    turret.readPeriodicInputs();
    hood.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    flywheel.writePeriodicOutputs();
    turret.writePeriodicOutputs();
    hood.writePeriodicOutputs();
  }

  public void forceStow(boolean forced) {
    forcedStow = forced;
  }

  private boolean followPlan = false;

  public void followPlan(boolean followPlan) {
    this.followPlan = followPlan;
  }
}
