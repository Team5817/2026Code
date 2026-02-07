package com.team5817.lib.drivers.Lights;


import org.littletonrobotics.junction.AutoLog;

import com.team5817.lib.drivers.Lights.LightsState.AnimationState;

import edu.wpi.first.wpilibj.util.Color;

public interface LightsIO {

  @AutoLog
  public static class LEDsIOInputs {
    public double brightness;
    public double frameWait;
    public double r;
    public double g;
    public double b;
  }


  //public default void updateInputs(ServoMotorIOInputs inputs) {}

  public default void setControl(LightsState.AnimationState state, double brightness, int frameRate){
    if (state == AnimationState.SOLID){
      setSolid();
    }
  }

  public default void updateInputs(){}

  public default void setSolid(){}

  public default void setRainbow(double brightness, int frameRate){}

  public default void setFade(double brightness, int frameRate){}

  public default void setRbgFade(double brightness, int frameRate){}

  public default void setStrobe(double brightness, int frameRate){}

  public default void setTwinkle(double brightness, int frameRate){}



  public default void stop() {}

  public default void start() {}

  
}
