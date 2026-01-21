package com.team5817.lib.requests;

public class NeverEndingRequest extends Request {

  @Override
  public void act() {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
