package com.team5817.frc2026;

import com.team5817.lib.drivers.Subsystem;
import edu.wpi.first.wpilibj.Timer;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Used to reset, start, stop, and update all subsystems at once. */
public class SubsystemManager {
  public static SubsystemManager mInstance = null;

  private List<Subsystem> mAllSubsystems;
  private double[] mCycleTimeAccumulator = new double[0];

  private SubsystemManager() {}

  /**
   * Returns the singleton instance of the SubsystemManager.
   *
   * @return the singleton instance of the SubsystemManager.
   */
  public static SubsystemManager getInstance() {
    if (mInstance == null) {
      mInstance = new SubsystemManager();
    }

    return mInstance;
  }

  /** Outputs telemetry data for all subsystems. */
  public void outputTelemetry() {
    if (RobotConstants.disableExtraTelemetry) {
      return;
    }
    mAllSubsystems.forEach(Subsystem::outputTelemetry);
  }

  /**
   * Checks the status of all subsystems.
   *
   * @return true if all subsystems are functioning correctly, false otherwise.
   */
  public boolean checkSubsystems() {
    boolean ret_val = true;

    for (Subsystem s : mAllSubsystems) {
      ret_val &= s.checkSystem();
    }

    return ret_val;
  }

  /** Stops all subsystems. */
  public void stop() {
    mAllSubsystems.forEach(Subsystem::stop);
  }

  /**
   * Returns the list of all subsystems.
   *
   * @return the list of all subsystems.
   */
  public List<Subsystem> getSubsystems() {
    return mAllSubsystems;
  }

  /**
   * Sets the list of all subsystems.
   *
   * @param allSubsystems the subsystems to be managed.
   */
  public void setSubsystems(Subsystem... allSubsystems) {
    mAllSubsystems = Arrays.asList(allSubsystems);
    mCycleTimeAccumulator = new double[mAllSubsystems.size()];
  }

  public void getFullCycleMS() {
    Arrays.fill(mCycleTimeAccumulator, 0.0);

    for (int i = 0; i < mAllSubsystems.size(); i++) {
      double start = Timer.getFPGATimestamp();
      mAllSubsystems.get(i).readPeriodicInputs();
      mCycleTimeAccumulator[i] += Timer.getFPGATimestamp() - start;
    }

    for (int i = 0; i < mAllSubsystems.size(); i++) {
      double start = Timer.getFPGATimestamp();
      mAllSubsystems.get(i).periodic();
      mCycleTimeAccumulator[i] += Timer.getFPGATimestamp() - start;
    }

    for (int i = 0; i < mAllSubsystems.size(); i++) {
      double start = Timer.getFPGATimestamp();
      mAllSubsystems.get(i).writePeriodicOutputs();
      mCycleTimeAccumulator[i] += Timer.getFPGATimestamp() - start;
    }

    logCycleTimes();
    outputTelemetry();
  }

  private void logCycleTimes() {
    for (int i = 0; i < mAllSubsystems.size(); i++) {
      Logger.recordOutput(
          "SubsystemManager/FullCycleMS/" + mAllSubsystems.get(i).getClass().getSimpleName(),
          mCycleTimeAccumulator[i] * 1000.0);
    }
  }

  public void start() {
    for (Subsystem s : mAllSubsystems) s.start();
  }
}