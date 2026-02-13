package com.team5817.frc2026.subsystems.Lights;

import com.team5817.lib.drivers.Subsystem;
import com.team5817.lib.drivers.Lights.LightsIO;
import com.team5817.lib.drivers.Lights.LightsState.LEDState;
import com.team5817.lib.requests.Request;


import org.littletonrobotics.junction.Logger;

public class Lights extends Subsystem {
  LEDState mState;
  LightsIO io;
  public Lights(LightsIO io) {
    this.io = io;
    setLeds(LEDState.RED);
  }

  public void setLeds(LEDState state){
    mState = state;
    io.setControl(state, LightsConstants.defaultFrameRate, 0, LightsConstants.maxSlot);
  }
  @Override
  public void outputTelemetry() {
      Logger.recordOutput("Lights/Main State", io.getState(mState));

  }

  public Request setState(LEDState state){
      return new Request() {
        @Override
        public void act() {
            setLeds(state);
        }
      };
  }
}

