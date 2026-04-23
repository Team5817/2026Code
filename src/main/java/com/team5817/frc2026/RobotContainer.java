package com.team5817.frc2026;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team5817.frc2026.generated.TunerConstants;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Drive.SwerveConstants;
import com.team5817.frc2026.subsystems.Indexer.Indexer;
import com.team5817.frc2026.subsystems.Indexer.IndexerConstants;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Intake.IntakeConstants;
import com.team5817.frc2026.subsystems.Lights.Lights;
import com.team5817.frc2026.subsystems.Shield.Shield;
import com.team5817.frc2026.subsystems.Shield.ShieldConstants;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Shooter.ShooterConstants;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.frc2026.subsystems.Vision.Vision;
import com.team5817.frc2026.subsystems.Vision.VisionConstants;
import com.team5817.lib.RobotMode;
import com.team5817.lib.drivers.Lights.LightsIOCANDLE;
import com.team5817.lib.drivers.Lights.LightsIOSim;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIOSim;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIOTalonFX;
import com.team5817.lib.drivers.Servos.ServoMotorIOSim;
import com.team5817.lib.drivers.Servos.ServoMotorIOTalonFX;
import com.team5817.lib.drivers.Vision.ManualVisionIOLimelight;
import com.team5817.lib.drivers.Vision.VisionIOLimelight;
import com.team5817.lib.drivers.Vision.VisionIOPhotonVisionSim;
import com.team5817.lib.swerve.GyroIOPigeon2;
import com.team5817.lib.swerve.GyroIOSim;
import com.team5817.lib.swerve.ModuleIOSim;
import com.team5817.lib.swerve.ModuleIOTalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  public Drive mDrive = null;
  public Intake mIntake = null;
  public Indexer mIndexer = null;
  public Shooter mShooter = null;
  public Vision mVision = null;
  public Shield mShield = null;
  public Lights mLight = null;
  public Superstructure mSuperstructure = null;

  public SwerveDriveSimulation driveSimulation = null;

  public RobotContainer() {
    switch (RobotMode.mode) {
      case REAL:
        makeRealRobot();
        break;

      default:
        break;
    }

    fillInSimulatedSubsytems();
    SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

    mSuperstructure = new Superstructure(mDrive, mIntake, mIndexer, mShooter, mShield, mLight);

    mSubsystemManager.setSubsystems(
        mDrive, mSuperstructure, mVision, mIntake, mShooter, mIndexer, mShield, mLight);
  }

  public void makeRealRobot() {
    mDrive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight),
            SwerveConstants.stabilizePID,
            SwerveConstants.snapPID);

    mIntake =
        new Intake(
            new RollerSubsystemIOTalonFX(
                Ports.INTAKE_ROLLERS, IntakeConstants.RollerConstants.kMotorConstants, 2.5),
            new ServoMotorIOTalonFX(IntakeConstants.DeployConstants.kRackLeftServoConstants),
            new ServoMotorIOTalonFX(IntakeConstants.DeployConstants.kRackRightServoConstants),
            () -> mDrive.getChassisSpeeds().vxMetersPerSecond);

    mIndexer =
        new Indexer(
            new RollerSubsystemIOTalonFX(Ports.SPINDEXER, IndexerConstants.kSpindexerConstants, 2),
            new RollerSubsystemIOTalonFX(Ports.TUNNEL, IndexerConstants.kTunnelConstants, 1));

    mShooter =
        new Shooter(
            new ServoMotorIOTalonFX(ShooterConstants.TurretConstants.kTurretServoConstants),
            new ServoMotorIOTalonFX(ShooterConstants.HoodConstants.kHoodServoConstants),
            new RollerSubsystemIOTalonFX(
                Ports.TURRET_FLYWHEEL1, ShooterConstants.kFlywheelConstants, 1),
            mDrive::getPose,
            mDrive::getChassisSpeeds,
            () -> 0.0);

    mVision =
        new Vision(
            mDrive::addVisionMeasurement,
            new ManualVisionIOLimelight(
                "limelight-turret",
                mShooter.getTurretCameraPoseSupplier(),
                () -> mDrive.getHeading(),
                false),
            new VisionIOLimelight("limelight-front", mDrive::getHeading),
            new VisionIOLimelight("limelight-back", mDrive::getHeading));

    mLight = new Lights(new LightsIOCANDLE(25));


    mShield = new Shield(new ServoMotorIOTalonFX(ShieldConstants.kShieldServoConstants));
  }

  public void wasteVision(Optional<Translation2d> gamepiecePoseMeters, double timestampSeconds) {}

  public void fillInSimulatedSubsytems() {

    if (mDrive == null)
      driveSimulation =
          new SwerveDriveSimulation(
              SwerveConstants.driveConfig, new Pose2d(3, 3, new Rotation2d()).wpi());
    if (mDrive == null) SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

    if (mShield == null)
      mShield = new Shield(new ServoMotorIOSim(ShieldConstants.kShieldServoConstants));

    if (mIntake == null)
      mIntake =
          new Intake(
              new RollerSubsystemIOSim(DCMotor.getKrakenX44(1), 1, 0.01),
              new ServoMotorIOSim(IntakeConstants.DeployConstants.kRackLeftServoConstants),
              new ServoMotorIOSim(IntakeConstants.DeployConstants.kRackRightServoConstants),
              () -> mDrive.getChassisSpeeds().vxMetersPerSecond);

    if (mIndexer == null)
      mIndexer =
          new Indexer(
              new RollerSubsystemIOTalonFX(
                  Ports.SPINDEXER, IndexerConstants.kSpindexerConstants, 1),
              new RollerSubsystemIOTalonFX(Ports.TUNNEL, IndexerConstants.kTunnelConstants, 1));

    if (mDrive == null)
      mDrive =
          new Drive(
              new GyroIOSim(driveSimulation.getGyroSimulation()),
              new ModuleIOSim(driveSimulation.getModules()[0]),
              new ModuleIOSim(driveSimulation.getModules()[1]),
              new ModuleIOSim(driveSimulation.getModules()[2]),
              new ModuleIOSim(driveSimulation.getModules()[3]),
              SwerveConstants.simStabilizePID,
              SwerveConstants.simSnapPID) {
            @Override
            public void simResetWorldPose(Pose2d newPose) {
              driveSimulation.setSimulationWorldPose(newPose.wpi());
            }
          };

    if (mIntake == null)
      mIntake =
          new Intake(
              new RollerSubsystemIOSim(DCMotor.getKrakenX44(1), 1, 0.01),
              new ServoMotorIOSim(IntakeConstants.DeployConstants.kRackLeftServoConstants),
              new ServoMotorIOSim(IntakeConstants.DeployConstants.kRackRightServoConstants),
              () -> mDrive.getChassisSpeeds().vxMetersPerSecond);

    if (mIndexer == null)
      mIndexer =
          new Indexer(
              new RollerSubsystemIOSim(DCMotor.getKrakenX44(1), 20, 10),
              new RollerSubsystemIOSim(DCMotor.getKrakenX44(1), 20, 10));

    if (mVision == null)
      mVision =
          new Vision(
              mDrive::addVisionMeasurement,
              new VisionIOPhotonVisionSim(
                  "limelight-up", VisionConstants.robotToCameraUp, this::getMapleSimPose),
              new VisionIOPhotonVisionSim(
                  "limelight-right", VisionConstants.robotToCameraLeft, this::getMapleSimPose),
              new VisionIOPhotonVisionSim(
                  "limelight-left", VisionConstants.robotToCameraRight, this::getMapleSimPose));

    if (mShooter == null)
      mShooter =
          new Shooter(
              new ServoMotorIOSim(ShooterConstants.TurretConstants.kTurretServoConstants),
              new ServoMotorIOSim(ShooterConstants.HoodConstants.kHoodServoConstants),
              new RollerSubsystemIOSim(DCMotor.getKrakenX60(2), 20, 10),
              mDrive::getPose,
              mDrive::getChassisSpeeds,
              mVision::timeSinceUpdate);

    if (mShield == null)
      mShield = new Shield(new ServoMotorIOSim(ShieldConstants.kShieldServoConstants));

    if (mLight == null) mLight = new Lights(new LightsIOSim());
  }

  private Pose2d getMapleSimPose() {
    return new Pose2d(driveSimulation.getSimulatedDriveTrainPose());
  }

  public void resetSimulation() {
    if (RobotMode.mode != RobotMode.Mode.SIM) return;
    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()).wpi());
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (RobotMode.mode != RobotMode.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
  }
}
