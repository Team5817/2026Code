package com.team5817.frc2026.autos.Actions;

import com.team5817.frc2026.subsystems.Elevator.Elevator;
import com.team5817.lib.requests.Request;
import com.team5817.lib.requests.RequestExecutor;

public class ClimbAction implements Action {
  Request climbRequest;
  RequestExecutor executor;

  public ClimbAction(Elevator c) {
    this.climbRequest = c.advanceClimbRequest();
    executor = new RequestExecutor();
  }

  @Override
  public boolean isFinished() {
    return executor.isFinished();
  }

  @Override
  public void update() {
    executor.update();
  }

  @Override
  public void done() {}

  @Override
  public void start() {
    executor.request(climbRequest);
  }
}
