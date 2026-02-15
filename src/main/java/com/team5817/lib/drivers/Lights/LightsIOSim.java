package com.team5817.lib.drivers.Lights;

import org.littletonrobotics.junction.Logger;


import com.ctre.phoenix6.hardware.CANdle;
import com.team5817.lib.drivers.Lights.LightsState.LEDState;

public class LightsIOSim implements LightsIO{
  CANdle mCandle;
  private LEDState state;
  private double frameRate;


  public LightsIOSim(){

  }

  @Override
  public void setControl(LightsState.LEDState state, double frameRate, int minSlot, int maxSlot) {
     this.state = state;
     this.frameRate = frameRate;
    Logger.recordOutput("Lights/Main State", this.state);
    Logger.recordOutput("Lights/Frame Rate", this.frameRate);
  }

}