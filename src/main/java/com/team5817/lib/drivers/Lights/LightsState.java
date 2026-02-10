package com.team5817.lib.drivers.Lights;

import edu.wpi.first.wpilibj.util.Color;

public class LightsState {
  public int blue;

	public static final Color RED = new Color(255, 0, 0);
	public static final Color PINK = new Color(255, 18, 143);
	public static final Color GREEN = new Color(0, 255, 8);
	public static final Color PURPLE = new Color(196, 18, 255);
	public static final Color ORANGE = new Color(255, 53, 13);
	public static final Color YELLOW = new Color(255, 150, 5);
	public static final Color CYAN = new Color(52, 155, 235);
	public static final Color BLUE = new Color(0, 0, 255);
  public static final Color WHITE = new Color(255,255,255);

  public static final Color OFF = ledOff();
  
  public static Color ledOff() {
		return new Color(0, 0, 0);
	}

  public enum LEDState{
    LARSON,
    RAINBOW,
    FIRE,
    SOLID,
    FLOW,
    RGBFADE,
    STROBE
  }
}