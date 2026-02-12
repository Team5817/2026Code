package com.team5817.lib.drivers.Lights;


import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.ModulateVBatOut;
import com.ctre.phoenix6.signals.Animation0TypeValue;
import com.team5817.lib.drivers.Lights.LightsState.LEDState;

import edu.wpi.first.wpilibj.util.Color;

public interface LightsIO {

  default void setControl(LightsState.LEDState state, double frameRate, int minSlot, int maxSlot){}
  
  default LEDState getState(LEDState state){return state;}

  default void stop() {}

  default void start() {}

}
