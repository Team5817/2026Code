package com.team5817.frc2026.subsystems.Shooter;

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
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;


public class Shooter extends Subsystem {

  @Getter private final Turret turret;
  @Getter private final Hood hood;
  @Getter private final RollerSubsystem<FlywheelState> flywheel;

  public Shooter(
      ServoMotorIO turretIO,
      ServoMotorIO hoodIO,
      RollerSubsystemIO flywheelIO,
      Supplier<Rotation2d> robotHeadingSupplier) {

   this.turret =
    new Turret(
        turretIO,
        () -> ShootingPlanner.getTurretAngle(ShootingTarget.HUB),
        () -> ShootingPlanner.getTurretVelocityFF(ShootingTarget.HUB),
        () -> ShootingPlanner.getTurretAngle(ShootingTarget.LOB),
        () -> ShootingPlanner.getTurretVelocityFF(ShootingTarget.LOB),
        robotHeadingSupplier);

this.hood =
    new Hood(
        hoodIO,
        () -> ShootingPlanner.getHoodAngle(ShootingTarget.HUB),
        () -> ShootingPlanner.getHoodVelocityFF(ShootingTarget.HUB),
        () -> ShootingPlanner.getHoodAngle(ShootingTarget.LOB),
        () -> ShootingPlanner.getHoodVelocityFF(ShootingTarget.LOB));


    FlywheelState.HUB.setSupplier(
        () -> ShootingPlanner.getFlywheelSpeed(ShootingTarget.HUB));

    FlywheelState.LOBBING.setSupplier(
        () -> ShootingPlanner.getFlywheelSpeed(ShootingTarget.LOB));

    flywheel =
        new RollerSubsystem<>(
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

  @Getter @Setter
  private State desiredState = State.STOW;

  private boolean forcedStow = false;
  private boolean followPlan = false;

  public enum State {
    STOW(Turret.State.STOW, Hood.State.STOW, FlywheelState.IDLE),
    CLOSE(Turret.State.STOW, Hood.State.CLOSE, FlywheelState.CLOSE),
    FAR(Turret.State.STOW, Hood.State.FAR, FlywheelState.FAR),
    HUB(Turret.State.HUB, Hood.State.HUB, FlywheelState.HUB),
    LOB(Turret.State.LOBBING, Hood.State.LOBBING, FlywheelState.LOBBING);

    final Turret.State turretState;
    final Hood.State hoodState;
    final FlywheelState flywheelState;

    State(
        Turret.State turretState,
        Hood.State hoodState,
        FlywheelState flywheelState) {
      this.turretState = turretState;
      this.hoodState = hoodState;
      this.flywheelState = flywheelState;
    }
  }

  @Override
  public void periodic() {
    if (forcedStow) {
      desiredState = State.STOW;
    } else if (followPlan) {
      desiredState = ShootingPlanner.recommendedShooterState();
    }

    turret.setState(desiredState.turretState);
    hood.setState(desiredState.hoodState);
    flywheel.setState(desiredState.flywheelState);

    if (isAtState() && !forcedStow) {
      mState = desiredState;
    }
  }

  @Override
  public void outputTelemetry() {
    Logger.recordOutput("Shooter/CurrentState", mState);
    Logger.recordOutput("Shooter/DesiredState", desiredState);
    Logger.recordOutput("Shooter/ForcedStow", forcedStow);
    Logger.recordOutput("Shooter/FollowPlan", followPlan);

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
        return mState == state;
      }
    };
  }

  public boolean isAtState(){
    return turret.atState() && hood.atState() && flywheel.atState();
  }
  @Override
  public void readPeriodicInputs() {
    turret.readPeriodicInputs();
    hood.readPeriodicInputs();
    flywheel.readPeriodicInputs();
  }

  @Override
  public void writePeriodicOutputs() {
    turret.writePeriodicOutputs();
    hood.writePeriodicOutputs();
    flywheel.writePeriodicOutputs();
  }

  public void forceStow(boolean forced) {
    forcedStow = forced;
  }

  public void followPlan(boolean enable) {
    followPlan = enable;
  }
}
