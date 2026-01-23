package com.team5817.frc2026.subsystems;

import com.team5817.frc2026.subsystems.Climb.Climb;
import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Intake.Intake;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
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
  public Climb mClimb;
  public Intake mIntake;

  @Setter private boolean allowAutoShoot = true;

  public Superstructure(Drive drive, Shooter shooter, Climb climb, Intake intake) {
    mDrive = drive;
    mShooter = shooter;
    mClimb = climb;
    mIntake = intake;
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

  @Override
  public void periodic() {
    requestExecutor.update();
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
