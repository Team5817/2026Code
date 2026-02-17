package com.team5817.frc2026.subsystems;

import com.team5817.frc2026.ActiveTracker;
import com.team5817.frc2026.subsystems.Climb.Climb;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Lights.Lights;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.frc2026.subsystems.Spindexer.SpindexerGroup;
import com.team5817.frc2026.subsystems.Stationary.FixedShooter;
import com.team5817.lib.drivers.Lights.LightsState.LEDState;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.NeverEndingRequest;
import com.team5817.lib.requests.Request;
import com.team5817.lib.requests.RequestExecutor;
import com.team5817.lib.requests.SequentialRequest;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends Subsystem {

  // Request tracking variables
  private RequestExecutor requestExecutor;

  // Subsystems
  public Drive mDrive;
  public Shooter mShooter;
  public FixedShooter mFixedShooter;
  public Intake mIntake;
  public SpindexerGroup mSpindexerGroup;
  public Climb mClimb;
  public Lights mLights;

  @Setter private boolean allowAutoShoot = true;

  public Superstructure(
      Drive drive,
      Intake intake,
      SpindexerGroup spindexerGroup,
      Shooter shooter,
      FixedShooter fixedShooter,
      Climb climb,
      Lights lights) {
    mDrive = drive;
    mIntake = intake;
    mSpindexerGroup = spindexerGroup;
    mShooter = shooter;
    mFixedShooter = fixedShooter;
    mClimb = climb;
    mLights = lights;
    this.requestExecutor = new RequestExecutor();
  }

  public Request CloseShotRequest() {
    return new SequentialRequest(
            mShooter.stateRequest(Shooter.State.CLOSE),
            // indexer on
            new NeverEndingRequest())
        .addName("Close Shot");
    // .withCleanup(
    //   () -> mIndexer.conformToState(Indexer.State.OFF)
    // );
  }

  public Request FarShotRequest() {
    return new SequentialRequest(
            mShooter.stateRequest(Shooter.State.FAR),
            // indexer on
            new NeverEndingRequest())
        .addName("FarShot");
    // .withCleanup(
    //   () -> mIndexer.conformToState(Indexer.State.OFF)
    // );
  }

  /*
          Idle: White
          Should Not Shoot: Yellow
          Should Shoot: Green
          Dual mode: Fire
          Climb: Purple
          Hopper empty: Flash Orange
          Alliance shift: Red or Blue
  */

  @Override
  public void periodic() {
    requestExecutor.update();
    if (ActiveTracker.getTimeToActive() % 2 == 0) {
      mLights.setLeds(LEDState.ORANGE);
    } else if (mClimb.getState() != Climb.State.ZERO) {
      mLights.setLeds(LEDState.CLIMBING);
    } else if (mShooter.getPlanner().shouldShoot()) {
      mLights.setLeds(LEDState.LOCKED);
    } else if (!mShooter.getPlanner().shouldShoot()) {
      mLights.setLeds(LEDState.NOT_LOCKED);
    }
    // else if (){
    //   mLights.setLeds(LEDState.FIRE);
    // }
    // else if (){
    //   mLights.setLeds(LEDState.STROBEORANGE);
    // }
    // else if (){ // alliance red
    //   mLights.setLeds(LEDState.RED);
    // }
    // else if (){ // alliance blue
    //   mLights.setLeds(LEDState.BLUE);
    // }
    else {
      mLights.setLeds(LEDState.NONE);
    }
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
