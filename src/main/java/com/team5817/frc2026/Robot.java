// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5817.frc2026;

import com.ctre.phoenix6.SignalLogger;
import com.team254.lib.swerve.ChassisSpeeds;
import com.team5817.BuildConstants;
import com.team5817.frc2026.autos.AutoBase;
import com.team5817.frc2026.autos.AutoExecuter;
import com.team5817.frc2026.autos.AutoModeFactory;
import com.team5817.frc2026.autos.TrajectoryLibrary.l;
import com.team5817.frc2026.controlboard.ControlBoard;
import com.team5817.frc2026.controlboard.DriverControls;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.lib.Elastic;
import com.team5817.lib.RobotMode;
import com.team5817.lib.Util;
import com.team5817.lib.requests.AutoShootRequest;
import com.team5817.lib.vision.LimelightPoseCalibrator;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private RobotContainer mRobotContainer;
  private SubsystemManager mSubsystemManager;
  private AutoExecuter mAutoExecuter;
  private AutoModeFactory mAutoModeFactory;
  DriverControls controls;
  ControlBoard controlBoard;

  Drive mDrive;

  public Robot() {
    super(0.02);
  }

  @SuppressWarnings("resource")
  /**
   * This method is called when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    if (Robot.isReal()) RobotMode.setMode(RobotMode.Mode.REAL);
    SignalLogger.enableAutoLogging(false);
    DriverStation.silenceJoystickConnectionWarning(true);
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight-right.local", port);
      PortForwarder.add(port + 10, "limelight-left.local", port);
      PortForwarder.add(port + 20, "limelight-up.local", port);
    }

    DriverStation.startDataLog(DataLogManager.getLog());

    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);

    if (RobotMode.isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      if (RobotMode.isReplay()) {
        setUseTiming(false); // Run as fast as possible
        String logPath =
            LogFileUtil
                .findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(
            new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new
        // log
        setUseTiming(false);
      } else {
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
      }
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.
    l.init();

    mRobotContainer = new RobotContainer();

    mDrive = mRobotContainer.mDrive;
    mAutoModeFactory = new AutoModeFactory(mRobotContainer.mSuperstructure, mDrive);
    mSubsystemManager = SubsystemManager.getInstance();

    Elastic.selectTab("Pre Match");

    controls = new DriverControls(mDrive, mRobotContainer.mSuperstructure);
    controlBoard = controls.mControlBoard;

    Logger.recordOutput("isComp", RobotConstants.isComp);
  }

  /** This method is called periodically, regardless of the robot's mode. */
  boolean needsZero = true;

  @Override
  public void robotPeriodic() {
    if (needsZero && DriverStation.getAlliance().isPresent()) {
      mDrive.allianceZeroGyro();
      needsZero = false;
    }
    Logger.recordOutput("Elastic/Match Time", Timer.getMatchTime());
    mSubsystemManager.updateSubsystems();
    RobotVisualizer.outputTelemetry();
    ActiveTracker.updateActive();
  }

  boolean disableGyroReset = false;

  /** This method is called once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    mSubsystemManager.start();
    neverEnabled = false;
    Elastic.selectTab("Autonomous");
    mAutoExecuter.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  boolean neverEnabled = true;

  /** This method is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    mSubsystemManager.start();
    neverEnabled = false;
    mDrive.setControlState(Drive.DriveControlState.OPEN_LOOP);

    Elastic.selectTab("Teleoperated");
    mDrive.stop();
    mRobotContainer.mIntake.conformToState(Intake.State.IDLE);
    mRobotContainer.mSuperstructure.request(
        new AutoShootRequest(mRobotContainer.mShooter.getPlanner(), mRobotContainer.mSuperstructure)
            .addName("AutoShoot"));
  }

  /** This method is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    controls.oneControllerMode();
    controlBoard.update();

    mDrive.feedTeleopSetpoint(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            controlBoard.getSwerveTranslation().x(),
            controlBoard.getSwerveTranslation().y(),
            controlBoard.getSwerveRotation(),
            Util.robotToFieldRelative(
                mDrive.getHeading(), DriverStation.getAlliance().get().equals(Alliance.Red))));
  }

  /** This method is called once each time the robot is disabled. */
  @Override
  public void disabledInit() {
    mRobotContainer.resetSimulation();
    mSubsystemManager.stop();

    if (mAutoExecuter != null) {
      mAutoExecuter.stop();
    }
    mAutoExecuter = new AutoExecuter();
  }

  /** This method is called periodically when the robot is disabled. */
  @Override
  public void disabledPeriodic() {
    l.update();
    // if(mVision.getMovingAverage().getSize()!=0&&neverEnabled)
    // mDrive.zeroGyro(mVision.getMovingAverage().getAverage());
    mAutoModeFactory.updateModeCreator();
    Optional<AutoBase> autoMode = mAutoModeFactory.getAutoMode();
    if (!autoMode.isPresent()) return;
    if (autoMode.get() != mAutoExecuter.getAuto()) mAutoExecuter.setAuto(autoMode.get());
  }

  LimelightPoseCalibrator mLeftLimelightPoseCalibrator;
  LimelightPoseCalibrator mRightLimelightPoseCalibrator;
  LimelightPoseCalibrator mUpLimelightPoseCalibrator;

  /** This method is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    mLeftLimelightPoseCalibrator = new LimelightPoseCalibrator("leftLimelightCalibration.json");
    mRightLimelightPoseCalibrator = new LimelightPoseCalibrator("rightLimelightCalibration.json");
    mUpLimelightPoseCalibrator = new LimelightPoseCalibrator("upLimelightCalibration.json");

    mLeftLimelightPoseCalibrator.start();
    mRightLimelightPoseCalibrator.start();
    mUpLimelightPoseCalibrator.start();
    // Elastic.selectTab("Systems Test");

    // mAutoExecuter.setAuto(new Characterize(mRobotContainer.mElevator, true));
    // mAutoExecuter.start();

  }

  /** This method is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    mLeftLimelightPoseCalibrator.update();
    mRightLimelightPoseCalibrator.update();
    mUpLimelightPoseCalibrator.update();
  }

  /** This method is called once when the simulation is initialized. */
  @Override
  public void simulationInit() {}

  /** This method is called periodically during simulation. */
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    mRobotContainer.displaySimFieldToAdvantageScope();
  }
}
