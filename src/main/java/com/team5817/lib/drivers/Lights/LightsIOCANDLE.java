package com.team5817.lib.drivers.Lights;

import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.ModulateVBatOut;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.team5817.frc2026.subsystems.Lights.LightsConstants;
import com.team5817.lib.drivers.Lights.LightsState.LEDState;

public class LightsIOCANDLE implements LightsIO{
  CANdle mCandle;
  public LightsIOCANDLE(int port){
    mCandle = new CANdle(port);  
  }
  @Override
  public void setControl(LightsState.LEDState state, double frameRate, int minSlot, int maxSlot) {
    switch (state) {
      case FLOW:
      mCandle.setControl(new ColorFlowAnimation(minSlot, maxSlot)
      .withUpdateFreqHz(frameRate)
      );
        break;

      case FIRE:
      mCandle.setControl(new FireAnimation(minSlot, maxSlot)
      .withUpdateFreqHz(frameRate)
      );
        break;

      case LARSON:
      mCandle.setControl(new LarsonAnimation(minSlot, maxSlot)
      .withUpdateFreqHz(frameRate)
      );
        break;

      case RAINBOW:
      mCandle.setControl(new RainbowAnimation(minSlot, maxSlot)
      .withUpdateFreqHz(frameRate)
      );
        break;

      case RGBFADE:
      mCandle.setControl(new RgbFadeAnimation(minSlot, maxSlot)
      .withUpdateFreqHz(frameRate)
      );
        break;

      case SOLID:
      mCandle.setControl(new SolidColor(minSlot, maxSlot)
      );
        break;
        
      case STROBE:
      mCandle.setControl(new StrobeAnimation(minSlot, maxSlot)
      .withUpdateFreqHz(frameRate)
      );
        break;

    
    }
  }
}