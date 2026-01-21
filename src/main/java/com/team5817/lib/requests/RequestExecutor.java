package com.team5817.lib.requests;

import lombok.Getter;

public class RequestExecutor {
  @Getter private Request currentRequest = null;
  private boolean startedCurrentRequest = false;

  public void request(Request r) {
    if (currentRequest != null) {
      currentRequest.cleanup();
    }
    currentRequest = r;
    startedCurrentRequest = false;
  }

  public void update() {
    if (currentRequest == null) {
      return;
    }

    if (!startedCurrentRequest && currentRequest.allowed()) {
      currentRequest.act();
      startedCurrentRequest = true;
    }

    if (startedCurrentRequest && currentRequest.isFinished()) {
      currentRequest = null;
    }
  }

  public boolean isFinished() {
    return currentRequest == null;
  }
}
