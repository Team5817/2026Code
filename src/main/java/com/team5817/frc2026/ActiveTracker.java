package com.team5817.frc2026;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import lombok.Getter;
import lombok.experimental.Accessors;
import org.littletonrobotics.junction.Logger;

public class ActiveTracker {
  private static Boolean wonAuto;

  @Getter
  @Accessors(prefix = "is")
  private static boolean isActive = false;

  // One-shot shake notifier state: remember if we've already signaled a shake
  // and the previous time-to-active so we only trigger once when crossing
  // the 5 second threshold.
  private static boolean hasShakenController = false;
  private static double previousTimeToActive = Double.POSITIVE_INFINITY;

  private enum Condition {
    ALWAYS_TRUE,
    WHEN_WON_AUTO,
    WHEN_LOST_AUTO
  }

  private static final class TeleopWindow {
    final double start; // inclusive (seconds remaining)
    final double end; // exclusive (seconds remaining)
    final Condition condition;

    TeleopWindow(double start, double end, Condition condition) {
      this.start = start;
      this.end = end;
      this.condition = condition;
    }

    boolean contains(double matchTime) {
      return matchTime >= start && matchTime < end;
    }

    boolean isActiveFor(boolean wonAuto) {
      switch (condition) {
        case ALWAYS_TRUE:
          return true;
        case WHEN_WON_AUTO:
          return wonAuto;
        case WHEN_LOST_AUTO:
          return !wonAuto;
        default:
          return false;
      }
    }
  }

  // windows described by [start, end) in seconds-remaining
  private static final TeleopWindow[] TELEOP_WINDOWS =
      new TeleopWindow[] {
        new TeleopWindow(130, Double.POSITIVE_INFINITY, Condition.ALWAYS_TRUE),
        new TeleopWindow(105, 130, Condition.WHEN_LOST_AUTO),
        new TeleopWindow(80, 105, Condition.WHEN_WON_AUTO),
        new TeleopWindow(55, 80, Condition.WHEN_LOST_AUTO),
        new TeleopWindow(30, 55, Condition.WHEN_WON_AUTO),
        new TeleopWindow(0, 30, Condition.ALWAYS_TRUE)
      };

  private static boolean ensureWonAuto() {
    if (wonAuto != null) return true;
    if (DriverStation.getAlliance().isEmpty()) return false;

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData != null && gameData.length() > 0) {
      wonAuto =
          gameData.charAt(0)
              == (DriverStation.getAlliance().get().equals(Alliance.Red) ? 'R' : 'B');
      return true;
    }
    return false;
  }

  public static double getTimeToActive() {
    if (DriverStation.isAutonomous()) return 0;
    if (!DriverStation.isTeleopEnabled()) return Double.POSITIVE_INFINITY;
    if (!ensureWonAuto()) return Double.POSITIVE_INFINITY;

    double matchTime = DriverStation.getMatchTime();

    // find current window
    TeleopWindow current = null;
    for (TeleopWindow w : TELEOP_WINDOWS) {
      if (w.contains(matchTime)) {
        current = w;
        break;
      }
    }

    // if current window is active for our alliance => already active
    if (current != null && current.isActiveFor(wonAuto)) return 0;

    // otherwise find next window (with smaller start) that will be active
    // windows are ordered from largest start to smallest start, so iterate and find first with
    // start < matchTime
    for (TeleopWindow w : TELEOP_WINDOWS) {
      if (w.start < matchTime && w.isActiveFor(wonAuto)) {
        return matchTime - w.start;
      }
    }

    return Double.POSITIVE_INFINITY;
  }

  public static void updateActive() {
    updateTelemetry();
    if (!ensureWonAuto()) {
      // can't decide yet; keep current state
      return;
    }

    if (DriverStation.isAutonomous()) {
      isActive = true;
      return;
    }

    if (!DriverStation.isTeleopEnabled()) {
      isActive = false;
      return;
    }

    double matchTime = DriverStation.getMatchTime();
    for (TeleopWindow w : TELEOP_WINDOWS) {
      if (w.contains(matchTime)) {
        isActive = w.isActiveFor(wonAuto);
        return;
      }
    }

    // fallback
    isActive = false;
  }

  public static boolean shouldShakeController() {
    double tta = getTimeToActive();

    boolean trigger = false;

    if (tta < 5.0 && previousTimeToActive >= 5.0 && !hasShakenController) {
      trigger = true;
      hasShakenController = true;
    }

    if (isActive || tta == Double.POSITIVE_INFINITY || tta >= 5.0) {
      hasShakenController = false;
    }

    previousTimeToActive = tta;
    return trigger;
  }

  public static void updateTelemetry() {
    Logger.recordOutput("ActiveTracker/Is Active", isActive);
    Logger.recordOutput("ActiveTracker/Won Auto", wonAuto != null ? wonAuto : false);
    Logger.recordOutput("ActiveTracker/Time To Active", getTimeToActive());
  }
}
