package com.team5817.frc2026.subsystems.Shooter;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.swerve.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import com.team5817.frc2026.RobotVisualizer;
import com.team5817.frc2026.planners.ShootingPlanner;
import com.team5817.frc2026.planners.ShootingTarget;
import com.team5817.frc2026.subsystems.Shooter.ShooterConstants.FlywheelState;
import com.team5817.lib.drivers.Rollers.RollerSubsystem;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.Request;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;


public class Shooter extends Subsystem {

  @Getter private final Turret turret;
  @Getter private final Hood hood;
  @Getter private final RollerSubsystem<ShooterConstants.FlywheelState> flywheel;

  @Getter private ShootingPlanner planner;

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

    // Use builder pattern to create ShootingPlanner
    this.planner =
        ShootingPlanner.builder()
            .shooterPoseSupplier(shooterPoseSupplier)
            .shooterVelocitySupplier(robotVelocitySupplier)
            .atStateSupplier(() -> atState && !forcedStow)
            .timeSinceVisionSupplier(timeSinceVision)
            .build();

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
  }

  /**
   * Returns a supplier that computes the turret-mounted camera pose expressed in robot frame.
   * The supplier is evaluated each call and reads the current turret position.
   */
  public Supplier<Pose3d> getTurretCameraPoseSupplier() {
    return () -> {
  // Turret getPosition() returns degrees. Apply sign multiplier for testing conventions.
  double turretYawRad = Math.toRadians(ShooterConstants.TURRET_YAW_SIGN * turret.getPosition());

      // Rotate the turret->cam offset by the turret yaw around robot z
      Translation3d turretToCamRotated =
          ShooterConstants.TurretToCam.rotateBy(new Rotation3d(0.0, 0.0, turretYawRad));

      // Combine robot->turret + rotated turret->cam to form robot->camera translation
      Translation3d cameraTranslation = ShooterConstants.robotToTurret.plus(turretToCamRotated);

  // Camera rotation: pitch from constants, yaw = turret yaw
  Rotation3d cameraRot = new Rotation3d(0.0, Math.toRadians(ShooterConstants.CAMERA_PITCH_DEGREES), turretYawRad);

      Pose3d pose = new Pose3d(cameraTranslation, cameraRot);
      Logger.recordOutput("Shooter/LL Pose", pose);
      return pose;
    };
  }

  @Getter
  @Accessors(prefix = "m")
  private State mState = State.STOW;

  @Getter @Setter private State desiredState = State.HUB;

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
