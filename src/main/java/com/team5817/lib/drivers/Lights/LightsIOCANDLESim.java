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

public class LightsIOCANDLESim implements LightsIO{
  CANdle mCandle;
  LEDState state;
  public LightsIOCANDLESim(int port){
    mCandle = new CANdle(port);  
  }
  @Override
  public void setControl(LightsState.LEDState state, double frameRate, int minSlot, int maxSlot) {
     this.state = state;
  }

  
}