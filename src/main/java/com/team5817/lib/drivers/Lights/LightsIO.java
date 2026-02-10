package com.team5817.lib.drivers.Lights;


import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.ModulateVBatOut;
import com.ctre.phoenix6.signals.Animation0TypeValue;
import com.team5817.lib.drivers.Lights.LightsState.LEDState;

import edu.wpi.first.wpilibj.util.Color;

public interface LightsIO {

  public default void setControl(LightsState.LEDState state, double frameRate, int minSlot, int maxSlot){}
  
  public default LEDState getState(){return  LEDState.SOLID;}

  public default void stop() {}

  public default void start() {}

  
}
