package com.team5817.frc2026;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.SynchronousPIDF;
import com.team5817.frc2026.generated.TunerConstants;
import com.team5817.frc2026.planners.ShootingPlanner;
import com.team5817.frc2026.subsystems.Superstructure;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Drive.SwerveConstants;
import com.team5817.frc2026.subsystems.GamePieceVision.GamepieceVision;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Intake.IntakeConstants;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Shooter.ShooterConstants;
import com.team5817.frc2026.subsystems.Vision.Vision;
import com.team5817.frc2026.subsystems.Vision.VisionConstants;
import com.team5817.lib.RobotMode;
import com.team5817.lib.drivers.Servos.ServoMotorIO;
import com.team5817.lib.drivers.Servos.ServoMotorIOSim;
import com.team5817.lib.drivers.Servos.ServoMotorIOTalonFX;
import com.team5817.lib.drivers.Vision.VisionIO;
import com.team5817.lib.drivers.Vision.VisionIOLimelight;
import com.team5817.lib.drivers.Vision.VisionIOPhotonVisionSim;
import com.team5817.lib.drivers.GamepieceVision.GamepieceVisionIO;
import com.team5817.lib.drivers.GamepieceVision.GamepieceVisionIOLimelight;
import com.team5817.lib.drivers.GamepieceVision.GamepieceVisionIOSim;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIO;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIOSim;
import com.team5817.lib.drivers.Rollers.RollerSubsystemIOTalonFX;
import com.team5817.lib.swerve.GyroIO;
import com.team5817.lib.swerve.GyroIOPigeon2;
import com.team5817.lib.swerve.GyroIOSim;
import com.team5817.lib.swerve.ModuleIO;
import com.team5817.lib.swerve.ModuleIOSim;
import com.team5817.lib.swerve.ModuleIOTalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class RobotContainer {
        public Drive mDrive = null;
        public Intake mIntake = null;
        public Shooter mShooter = null;
        public Vision mVision = null;
        public GamepieceVision mGamepieceVision = null;
        public Superstructure mSuperstructure = null;

        public SwerveDriveSimulation driveSimulation = null;

        public RobotContainer() {
                switch (RobotMode.mode) {
                        case REAL:
                                makeRealRobot();
                                break;
                        case SIM:
                                makeSimulatedRobot();
                                break;
                        default:
                                break;
                }

                // makeEmptyRobot();

                SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

                mSuperstructure = new Superstructure(
                                mDrive,mShooter);

                mSubsystemManager.setSubsystems(
                                mDrive,
                                mSuperstructure,
                                mVision,
                                mGamepieceVision);

        }

        public void makeRealRobot() {

                
                
                

                /*needs to be rafactored */
                mIntake = new Intake(
                                new RollerSubsystemIOTalonFX(Ports.INTAKE_ROLLERS,
                                                IntakeConstants.RollerConstants.motorConstants, 1),
                                new RollerSubsystemIOTalonFX(Ports.INTAKE_CANCODER,
                                                IntakeConstants.RollerConstants.motorConstants, 1),
                                new RollerSubsystemIOTalonFX(Ports.SPINDEXER_1,
                                                IntakeConstants.RollerConstants.motorConstants, 1),
                                new ServoMotorIOTalonFX(IntakeConstants.DeployConstants.kDeployServoConstants));

                mDrive = new Drive(
                                new GyroIOPigeon2(),
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight),
                                SwerveConstants.stabilizePID,
                                SwerveConstants.snapPID);

                mShooter = new Shooter(
                        new ServoMotorIOTalonFX(ShooterConstants.TurretConstants.kTurretServoConstants),
                        new ServoMotorIOTalonFX(ShooterConstants.HoodConstants.kHoodServoConstants),
                        new RollerSubsystemIOTalonFX(Ports.FLYWHEEL_1, ShooterConstants.flywheelConstants, 1),
                        new ShootingPlanner(
                                null,
                         null,
                          null,
                           null));//TODO: pass real suppliers

                mVision = new Vision(
                                mDrive::addVisionMeasurement,
                                // new VisionIOLimelight("limelight-up", mDrive::getHeading),
                                new VisionIOLimelight("limelight-left", mDrive::getHeading),
                                new VisionIOLimelight("limelight-right", mDrive::getHeading));


                mGamepieceVision = new GamepieceVision(
                                this::wasteVision,
                                mDrive::getPose,
                                new GamepieceVisionIOLimelight(
                                                "Limelight-back",
                                                VisionConstants.robotToCameraBack,
                                                Units.inchesToMeters(2)));     
        }

        public void wasteVision(Optional<Translation2d> gamepiecePoseMeters, double timestampSeconds) {}

        public void makeSimulatedRobot() {
                driveSimulation = new SwerveDriveSimulation(SwerveConstants.driveConfig, new Pose2d(3, 3, new Rotation2d()).wpi());
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                mIntake = new Intake(
                                new RollerSubsystemIOSim(DCMotor.getKrakenX60(1), 1, 0.01),
                                new RollerSubsystemIOSim(DCMotor.getKrakenX60(1), 1, 0.01),
                                new RollerSubsystemIOSim(DCMotor.getKrakenX60(1), 1, 0.01),
                                new ServoMotorIOSim(IntakeConstants.DeployConstants.kDeployServoConstants));

                mDrive = new Drive(
                                new GyroIOSim(driveSimulation.getGyroSimulation()),
                                new ModuleIOSim(driveSimulation.getModules()[0]),
                                new ModuleIOSim(driveSimulation.getModules()[1]),
                                new ModuleIOSim(driveSimulation.getModules()[2]),
                                new ModuleIOSim(driveSimulation.getModules()[3]),
                                SwerveConstants.simStabilizePID,
                                SwerveConstants.simSnapPID){
                                        @Override
                                        public void simResetWorldPose(Pose2d newPose) {
                                            driveSimulation.setSimulationWorldPose(newPose.wpi());
                                        }
                                };

                mVision = new Vision(
                                mDrive::addVisionMeasurement,
                                new VisionIOPhotonVisionSim("limelight-up", VisionConstants.robotToCameraUp,
                                                this::getMapleSimPose),
                                new VisionIOPhotonVisionSim("limelight-right", VisionConstants.robotToCameraLeft,
                                                this::getMapleSimPose),
                                new VisionIOPhotonVisionSim("limelight-left", VisionConstants.robotToCameraRight,
                                                this::getMapleSimPose));

                mGamepieceVision = new GamepieceVision(
                                this::wasteVision,
                                this::getMapleSimPose,
                                new GamepieceVisionIOSim(
                                        this::getMapleSimPose,
                                        SimulatedArena.getInstance()
                                        ));
        }
        private Pose2d getMapleSimPose(){
                return new Pose2d(driveSimulation.getSimulatedDriveTrainPose());
        }

        public void makeEmptyRobot() {

                if(mIntake == null)
                        mIntake = new Intake(
                                new RollerSubsystemIO() {
                                },
                                new RollerSubsystemIO() {
                                },
                                new RollerSubsystemIO() {
                                },
                                new ServoMotorIO() {
                                });
                

                if(mDrive == null)
                        mDrive = new Drive(
                                new GyroIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                },
                                new SynchronousPIDF(),
                                new SynchronousPIDF());

                if(mVision == null)
                        mVision = new Vision(
                                mDrive::addVisionMeasurement,
                                new VisionIO() {
                                },
                                new VisionIO() {
                                },
                                new VisionIO() {
                                });

                if(mGamepieceVision == null)
                        mGamepieceVision = new GamepieceVision(
                                this::wasteVision,
                                mDrive::getPose,
                                new GamepieceVisionIO() {
                                });
        }


     public void resetSimulation() {
        if (RobotMode.mode != RobotMode.Mode.SIM) return;
        driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()).wpi());
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void displaySimFieldToAdvantageScope() {
        if (RobotMode.mode != RobotMode.Mode.SIM) return;

        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral",
                SimulatedArena.getInstance().getGamePiecesByType("Coral").toArray(new Pose3d[0]));
        Logger.recordOutput(
                "FieldSimulation/Algae",
                SimulatedArena.getInstance().getGamePiecesByType("Algae").toArray(new Pose3d[0]));
    }
}
