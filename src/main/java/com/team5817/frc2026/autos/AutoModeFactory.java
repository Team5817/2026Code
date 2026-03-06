package com.team5817.frc2026.autos;

import com.team5817.frc2026.autos.Modes.DNRD;
import com.team5817.frc2026.autos.Modes.DNS;
import com.team5817.frc2026.autos.Modes.DoNothingMode;
import com.team5817.frc2026.autos.Modes.H;
import com.team5817.frc2026.autos.Modes.NRH;
import com.team5817.frc2026.autos.Modes.NS;
import com.team5817.frc2026.autos.Modes.NSH;
import com.team5817.frc2026.autos.Modes.NSwipe;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Superstructure;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/** This class is responsible for selecting the autonomous mode for the robot. */
public class AutoModeFactory {
  private final Drive d;
  private final Superstructure s;

  public enum DesiredMode {
    DO_NOTHING,
    H,
    NS,
    NSH,
    NRH,
    DNS,
    NSWIPE,
    D,
    DNRD,
    HNRD,
    NRHT
  }

  public enum StartingPosition {
    TRENCH_H(
        DesiredMode.DO_NOTHING,
        DesiredMode.H,
        DesiredMode.NS,
        DesiredMode.NSH,
        DesiredMode.NRH,
        DesiredMode.NSWIPE,
        DesiredMode.HNRD,
        DesiredMode.NRHT),
    TRENCH_D(DesiredMode.DO_NOTHING, DesiredMode.D, DesiredMode.DNS, DesiredMode.DNRD),
    CENTER(DesiredMode.DO_NOTHING);

    public List<DesiredMode> modes;

    private StartingPosition(DesiredMode... validModes) {
      this.modes = List.of(validModes);
    }
  }

  public enum ClimbSelection {
    SHOULD_CLIMB,
    SHOULD_NOT_CLIMB
  }

  private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;
  private StartingPosition mCachedStartingPosition = StartingPosition.TRENCH_H;
  private ClimbSelection mCachedClimbSelection = ClimbSelection.SHOULD_CLIMB;

  private Optional<AutoBase> mAutoMode = Optional.empty();

  private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();
  private static SendableChooser<StartingPosition> mStartingPositionSelector =
      new SendableChooser<>();
  private static SendableChooser<ClimbSelection> mClimbPreferenceSelector = new SendableChooser<>();

  /**
   * Constructor for AutoModeSelector. Initializes the SendableChoosers for starting position,
   * pickup location, and scoring locations.
   */
  public AutoModeFactory(Superstructure s, Drive d) {
    this.s = s;
    this.d = d;

    mStartingPositionSelector.setDefaultOption("TRENCH_D", StartingPosition.TRENCH_D);
    mStartingPositionSelector.addOption("TRENCH_H", StartingPosition.TRENCH_H);
    mStartingPositionSelector.addOption("CENTER", StartingPosition.CENTER);

    mClimbPreferenceSelector.setDefaultOption("DO NOT CLIMB", ClimbSelection.SHOULD_NOT_CLIMB);
    mClimbPreferenceSelector.addOption("CLIMB", ClimbSelection.SHOULD_CLIMB);
  }

  /**
   * Updates the mode creator based on the selected starting position and desired mode. Updates the
   * cached values for pickup location and scoring locations.
   */
  public void updateModeCreator() {
    if (mCachedStartingPosition != mStartingPositionSelector.getSelected()
        && mStartingPositionSelector.getSelected() != null) {
      mModeChooser = new SendableChooser<>();
      mStartingPositionSelector
          .getSelected()
          .modes
          .forEach(m -> mModeChooser.addOption(m.name(), m));
      mCachedStartingPosition = mStartingPositionSelector.getSelected();
    }

    DesiredMode desiredMode = mModeChooser.getSelected();

    if (desiredMode == null) {
      desiredMode = DesiredMode.DO_NOTHING;
    }
    mAutoMode = getAutoModeForParams(desiredMode);

    mCachedClimbSelection = mClimbPreferenceSelector.getSelected();

    SmartDashboard.putData("Starting Position", mStartingPositionSelector);
    SmartDashboard.putData("Auto Mode", mModeChooser);
    Logger.recordOutput("Selected Auto", desiredMode);

    SmartDashboard.putData("Climb Preference", mClimbPreferenceSelector);
  }

  /**
   * Returns the AutoBase instance for theputp given desired mode.
   *
   * @param mode The desired autonomous mode.
   * @return An Optional containing the AutoBase instance if a valid mode is found, otherwise an
   *     empty Optional.
   */
  private Optional<AutoBase> getAutoModeForParams(DesiredMode mode) {
    switch (mode) {
      case DO_NOTHING:
        return Optional.of(new DoNothingMode());
      case H:
        return Optional.of(new H(s, mCachedClimbSelection));
      case NS:
        return Optional.of(new NS(s, mCachedClimbSelection));
      case NSH:
        return Optional.of(new NSH(s, mCachedClimbSelection));
      case NRH:
        return Optional.of(new NRH(s, mCachedClimbSelection));
      case DNS:
        return Optional.of(new DNS(s, mCachedClimbSelection));
      case NSWIPE:
        return Optional.of(new NSwipe(s, mCachedClimbSelection));
      case DNRD:
        return Optional.of(new DNRD(s, mCachedClimbSelection));
      default:
        System.out.println("ERROR: unexpected auto mode: " + mode);
        break;
    }

    System.err.println("No valid auto mode found for  " + mode);
    return Optional.empty();
  }

  /**
   * Returns the SendableChooser for selecting the desired mode.
   *
   * @return The SendableChooser for desired mode.
   */
  public static SendableChooser<DesiredMode> getModeChooser() {
    return mModeChooser;
  }

  /**
   * Returns the cached desired autonomous mode.
   *
   * @return The cached desired mode.
   */
  public DesiredMode getDesiredAutomode() {
    return mCachedDesiredMode;
  }

  /** Resets the AutoModeSelector by clearing the cached auto mode and desired mode. */
  public void reset() {
    mAutoMode = Optional.empty();
    mCachedDesiredMode = null;
  }

  /** Outputs the selected autonomous mode and starting position to the SmartDashboard. */
  public void outputToSmartDashboard() {
    SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
    SmartDashboard.putString("Starting Position Selected", mCachedStartingPosition.name());
  }

  /**
   * Returns the currently selected AutoBase instance.
   *
   * @return An Optional containing the AutoBase instance if present, otherwise an empty Optional.
   */
  public Optional<AutoBase> getAutoMode() {

    if (!mAutoMode.isPresent()) {
      return Optional.empty();
    }

    return mAutoMode;
  }
}
