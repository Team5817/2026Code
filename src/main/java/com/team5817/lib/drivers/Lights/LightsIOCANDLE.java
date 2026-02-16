package com.team5817.lib.drivers.Lights;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

public class LightsIOCANDLE implements LightsIO{
  CANdle mCandle;
  public LightsIOCANDLE(int port){
    mCandle = new CANdle(port);  
  }
  @Override
  public void setControl(LightsState.LEDState state, double frameRate, int minSlot, int maxSlot) {
    switch (state.animation) {
      case "FLOW":
      mCandle.setControl(new ColorFlowAnimation(minSlot, maxSlot)
      .withUpdateFreqHz(frameRate)
      );
        break;

      case "FIRE":
      mCandle.setControl(new FireAnimation(minSlot, maxSlot)
      .withUpdateFreqHz(frameRate)
      );
        break;

      case "LARSON":
      mCandle.setControl(new LarsonAnimation(minSlot, maxSlot)
      .withUpdateFreqHz(frameRate)
      );
        break;

      case "RAINBOW":
      mCandle.setControl(new RainbowAnimation(minSlot, maxSlot)
      .withUpdateFreqHz(frameRate)
      );
        break;

      case "RGBFADE":
      mCandle.setControl(new RgbFadeAnimation(minSlot, maxSlot)
      .withUpdateFreqHz(frameRate)
      );
        break;

        
      case "STROBE":
      mCandle.setControl(new StrobeAnimation(minSlot, maxSlot)
      .withUpdateFreqHz(frameRate)
      );
        break;
      case "SOLID":
        if(state.color!=null)
          mCandle.setControl(new SolidColor(minSlot, maxSlot).withColor(new RGBWColor(state.color)));
        break;
    
    }
  }

}