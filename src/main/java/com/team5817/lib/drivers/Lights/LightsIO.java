package com.team5817.lib.drivers.Lights;

import com.team5817.lib.drivers.Lights.LightsState.LEDState;


public interface LightsIO {

  default void setControl(LightsState.LEDState state, double frameRate, int minSlot, int maxSlot){}
  
  default LEDState getState(LEDState state){return state;}

  default void stop() {}

  default void start() {}

}
