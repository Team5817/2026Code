package com.team5817.frc2026.autos;

import com.team5817.frc2026.autos.Modes.CD;
import com.team5817.frc2026.autos.Modes.D;
import com.team5817.frc2026.autos.Modes.DoNothingMode;
import com.team5817.frc2026.autos.Modes.MT;
import com.team5817.frc2026.autos.Modes.MTC;
import com.team5817.frc2026.autos.Modes.MTB;
import com.team5817.frc2026.autos.Modes.PL;
import com.team5817.frc2026.autos.Modes.PLL;
import com.team5817.frc2026.autos.Modes.PLR;
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
    CLOSE_SWEEP,
    MT_SCOOP,
    MT_COUNTER,
    MT_BUMP,
    D,
    CD,
    PL,
    PL_LEFT,
    PL_RIGHT
  }

  public enum StartingSelection {
    TRENCH_H(
        DesiredMode.DO_NOTHING,
        DesiredMode.CLOSE_SWEEP,
        DesiredMode.MT_SCOOP,
        DesiredMode.MT_COUNTER,
        DesiredMode.MT_BUMP),
    TRENCH_D(
        DesiredMode.DO_NOTHING,
        DesiredMode.CLOSE_SWEEP,
        DesiredMode.MT_SCOOP,
        DesiredMode.D,
        DesiredMode.MT_COUNTER,
        DesiredMode.MT_BUMP),
    CENTER(
        DesiredMode.DO_NOTHING,
        DesiredMode.CD,
        DesiredMode.PL,
        DesiredMode.PL_LEFT,
        DesiredMode.PL_RIGHT);

    public List<DesiredMode> modes;

    private StartingSelection(DesiredMode... validModes) {
      this.modes = List.of(validModes);
    }
  }

  private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;
  private StartingSelection mCachedStartingSelection = StartingSelection.TRENCH_H;

  private Optional<AutoBase> mAutoMode = Optional.empty();

  private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();
  private static SendableChooser<StartingSelection> mStartingPositionSelector =
      new SendableChooser<>();

  /**
   * Constructor for AutoModeSelector. Initializes the SendableChoosers for starting position,
   * pickup location, and scoring locations.
   */
  public AutoModeFactory(Superstructure s, Drive d) {
    this.s = s;
    this.d = d;

    mStartingPositionSelector.setDefaultOption("TRENCH_D", StartingSelection.TRENCH_D);
    mStartingPositionSelector.addOption("TRENCH_H", StartingSelection.TRENCH_H);
    mStartingPositionSelector.addOption("CENTER", StartingSelection.CENTER);
  }

  /**
   * Updates the mode creator based on the selected starting position and desired mode. Updates the
   * cached values for pickup location and scoring locations.
   */
  public void updateModeCreator() {
    if (mCachedStartingSelection != mStartingPositionSelector.getSelected()
        && mStartingPositionSelector.getSelected() != null) {
      mModeChooser = new SendableChooser<>();
      mStartingPositionSelector
          .getSelected()
          .modes
          .forEach(m -> mModeChooser.addOption(m.name(), m));
      mCachedStartingSelection = mStartingPositionSelector.getSelected();
    }

    DesiredMode desiredMode = mModeChooser.getSelected();

    if (desiredMode == null) {
      desiredMode = DesiredMode.DO_NOTHING;
    }
    mAutoMode = getAutoModeForParams(desiredMode);

    SmartDashboard.putData("Starting Position", mStartingPositionSelector);
    SmartDashboard.putData("Auto Mode", mModeChooser);
    Logger.recordOutput("Selected Auto", desiredMode);
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

      case MT_SCOOP:
        return Optional.of(new MT(s, mCachedStartingSelection == StartingSelection.TRENCH_H, true));

      case MT_COUNTER:
        return Optional.of(
            new MTC(s, mCachedStartingSelection == StartingSelection.TRENCH_H, true));

      case MT_BUMP:
        return Optional.of(
          new MTB(s, mCachedStartingSelection == StartingSelection.TRENCH_H, true));

      case CD:
        return Optional.of(new CD(s));

      case D:
        return Optional.of(new D(s));

      case PL:
        return Optional.of(new PL(s));

      case PL_LEFT:
        return Optional.of(new PLL(s));

      case PL_RIGHT:
        return Optional.of(new PLR(s));

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
    SmartDashboard.putString("Starting Position Selected", mCachedStartingSelection.name());
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
