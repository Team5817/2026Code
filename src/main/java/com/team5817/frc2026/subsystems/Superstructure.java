package com.team5817.frc2026.subsystems;

import com.team5817.frc2026.ActiveTracker;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Indexer.Indexer;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Lights.Lights;
import com.team5817.frc2026.subsystems.Shield.Shield;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.lib.drivers.Lights.LightsState.LEDState;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.NeverEndingRequest;
import com.team5817.lib.requests.ParallelRequest;
import com.team5817.lib.requests.Request;
import com.team5817.lib.requests.RequestExecutor;
import com.team5817.lib.requests.SequentialRequest;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends Subsystem {

  // Request tracking variables
  private RequestExecutor requestExecutor;

  // Subsystems
  public Drive mDrive;
  public Shooter mShooter;
  public Intake mIntake;
  public Indexer mIndexer;
  public Shield mShield;
  public Lights mLights;

  @Setter private boolean allowAutoShoot = true;

  public Superstructure(
      Drive drive,
      Intake intake,
      Indexer spindexerGroup,
      Shooter shooter,
      Shield shield,
      Lights lights) {
    mDrive = drive;
    mIntake = intake;
    mIndexer = spindexerGroup;
    mShooter = shooter;
    mShield = shield;
    mLights = lights;
    this.requestExecutor = new RequestExecutor();
  }

  public Request CloseShotRequest() {
    return new SequentialRequest(
            new ParallelRequest(
                mShooter.stateRequest(Shooter.State.CLOSE),
                mIndexer.stateRequest(Indexer.State.FEED)),
            new NeverEndingRequest())
        .addName("CloseShot");
  }

  public Request FarShotRequest() {
    return new SequentialRequest(
            new ParallelRequest(
                mShooter.stateRequest(Shooter.State.FAR),
                mIndexer.stateRequest(Indexer.State.FEED)),
            new NeverEndingRequest())
        .addName("FarShot");
  }

  @Override
  public void periodic() {
    requestExecutor.update();
    handleLED();
  }

  public void handleLED() {
    if (DriverStation.isDisabled()) {
      mLights.setLeds(LEDState.NONE);
      return;
    }

    if (ActiveTracker.getShiftInfo().active()) {
      if (ActiveTracker.getShiftInfo().remainingTime() < 5) mLights.setLeds(LEDState.BLINK_BLUE);
      else {
        mLights.setLeds(LEDState.BLUE);
      }
      return;
    } else if (ActiveTracker.getShiftInfo().remainingTime() < 5) {
      mLights.setLeds(LEDState.RED);
      return;
    }
    mLights.setLeds(LEDState.TEAL);
  }

  public boolean requestsCompleted() {
    return this.requestExecutor.isFinished();
  }

  public void request(Request r) {
    requestExecutor.request(r);
  }

  @Override
  public void outputTelemetry() {
    if (!requestsCompleted())
      Logger.recordOutput("Active Request", requestExecutor.getCurrentRequest().getName());
    else Logger.recordOutput("Active Request", "null");
  }
}
