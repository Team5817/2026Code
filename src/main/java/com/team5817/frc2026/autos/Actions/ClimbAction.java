package com.team5817.frc2026.autos.Actions;

import com.team5817.frc2026.subsystems.Climb.Climb;
import com.team5817.lib.requests.Request;
import com.team5817.lib.requests.RequestExecutor;

public class ClimbAction implements Action {
  Request climbRequest;
  RequestExecutor executor;

  public ClimbAction(Climb c) {
    this.climbRequest = c.climbRequest();
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
