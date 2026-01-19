package com.team5817.frc2026.subsystems;

import com.team5817.frc2026.subsystems.Drive.Drive;
import com.team5817.frc2026.subsystems.Shooter.Shooter;
import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.requests.AutoShootRequest;
import com.team5817.lib.requests.IfRequest;
import com.team5817.lib.requests.Request;
import com.team5817.lib.requests.SequentialRequest;

import lombok.Setter;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends Subsystem {

  // Request tracking variables
  private Request activeRequest = null;
  private ArrayList<Request> queuedRequests = new ArrayList<>(0);
  private boolean hasNewRequest = false;
  private boolean allRequestsComplete = false;
  

  // Subsystems
  public Drive mDrive;
  public Shooter mShooter;

  @Setter
  private boolean allowAutoShoot = true;
  /**
   * Constructor for the Superstructure class.
   */
  public Superstructure(Drive drive, Shooter shooter) {
    mDrive = drive;
    mShooter = shooter;
  }


  public Request CloseShotRequest(){
    return new SequentialRequest(
      mShooter.stateRequest(Shooter.State.CLOSE),
      //indexer on
      forever()
    ).withCleanup(
      () -> mIndexer.conformToState(Indexer.State.OFF)
    );
  }

  public Request FarShotRequest(){
    return new SequentialRequest(
      mShooter.stateRequest(Shooter.State.FAR),
      //indexer on
      forever()
    ).withCleanup(
      () -> mIndexer.conformToState(Indexer.State.OFF)
    );
  }

  @Override
  public void periodic() {
    manageRequests();
    if(allRequestsComplete && allowAutoShoot){
      request(new AutoShootRequest(mShooter.getPlanner(), this));
    }
  }


  /**
   * Checks if all requests have been completed.
   * 
   * @return True if all requests are complete, false otherwise.
   */
  public boolean requestsCompleted() {
    return allRequestsComplete;
  }

  /**
   * Sets a new active request and clears the request queue.
   * 
   * @param r The new request to be set as active.
   */
  public void request(Request r) {
    setActiveRequest(r);
    clearRequestQueue();
  }

  /**
   * Sets the active request.
   * 
   * @param request The request to be set as active.
   */
  public void setActiveRequest(Request request) {
    activeRequest = request;
    hasNewRequest = true;
    allRequestsComplete = false;
  }

  /**
   * Clears the request queue.
   */
  public void clearRequestQueue() {
    queuedRequests.clear();
  }

  /**
   * Sets the request queue with a list of requests.
   * 
   * @param requests The list of requests to be added to the queue.
   */
  public void setRequestQueue(List<Request> requests) {
    clearRequestQueue();
    for (Request req : requests) {
      queuedRequests.add(req);
    }
  }

  /**
   * Sets the request queue with an active request and a list of requests.
   * 
   * @param activeRequest The active request to be set.
   * @param requests      The list of requests to be added to the queue.
   */
  public void setRequestQueue(Request activeRequest, ArrayList<Request> requests) {
    request(activeRequest);
    setRequestQueue(requests);
  }

  /**
   * Adds a request to the queue.
   * 
   * @param req The request to be added to the queue.
   */
  public void addRequestToQueue(Request req) {
    queuedRequests.add(req);
  }

  public void manageRequests() {
    try {
      if (hasNewRequest && activeRequest != null) {
        activeRequest.act();
        hasNewRequest = false;
      }

      if (activeRequest == null) {
        if (queuedRequests.isEmpty()) {
          allRequestsComplete = true;
        } else {
          request(queuedRequests.remove(0));
        }
      } else if (activeRequest.isFinished()) {
        activeRequest = null;
      }

    } catch (Exception e) {
      e.printStackTrace();
    }
  }
  /**
   * Creates a request that runs indefinitely.
   * 
   * @return A request that never finishes.
   */
  public Request forever() {
    return new Request() {
      @Override
      public void act() {
      }

      @Override
      public boolean isFinished() {
        return false;
      }
    };
  }

  @Override
  public void stop() {
    activeRequest = null;
    clearRequestQueue();
  }

  @Override
  public boolean checkSystem() {
    return false;
  }


  @Override
  public void outputTelemetry() {
    Logger.recordOutput("Active Request", activeRequest.getName());
  }

  public Request BooleanWaitRequest(BooleanSupplier booleanSupplier, boolean target) {
    return new Request() {
      @Override
      public void act() {
      }

      @Override
      public boolean isFinished() {
        return target ? booleanSupplier.getAsBoolean() : !booleanSupplier.getAsBoolean();
      }
    };
  }

}
