package com.team5817.frc2026.subsystems.Drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.config.RobotConfig;
import com.team254.lib.util.SynchronousPIDF;
import com.team5817.frc2026.generated.TunerConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public final class SwerveConstants {

  public static RobotConfig mRobotConfig;

  static {
    try {
      mRobotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  /* Controller Invert */
  public static final boolean invertYAxis = false;
  public static final boolean invertRAxis = true;
  public static final boolean invertXAxis = false;

  /* Heading Controller */
  public static final SynchronousPIDF stabilizePID = new SynchronousPIDF(20.0, 0.0, 0.3);

  public static final SynchronousPIDF snapPID = new SynchronousPIDF(1.5, 0.0, 0.8, 1.0);

  public static final SynchronousPIDF simStabilizePID = new SynchronousPIDF(16.0, 0.0, 0.6);

  public static final SynchronousPIDF simSnapPID = new SynchronousPIDF(6.0, 0.0, 0.8, 2.0);

  public static final double kTrajectoryDeadband = .05;

  public static final SwerveModuleSimulationConfig moduleConfig =
      new SwerveModuleSimulationConfig(
          DCMotor.getKrakenX60(1),
          DCMotor.getFalcon500(1),
          TunerConstants.FrontLeft.DriveMotorGearRatio,
          TunerConstants.FrontLeft.SteerMotorGearRatio,
          Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
          Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
          Inches.of(2),
          KilogramSquareMeters.of(0.02),
          1.2);

  public static final DriveTrainSimulationConfig driveConfig =
      DriveTrainSimulationConfig.Default()
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(() -> new SwerveModuleSimulation(moduleConfig))
          .withBumperSize(Meters.of(.7), Meters.of(.7))
          .withCustomModuleTranslations(Drive.getModuleTranslations())
          .withRobotMass(Pounds.of(115));
}
