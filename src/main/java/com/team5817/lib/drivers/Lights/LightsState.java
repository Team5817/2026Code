package com.team5817.lib.drivers.Lights;

import edu.wpi.first.wpilibj.util.Color;

public class LightsState {
  public int blue;

	public static final Color RED = new Color(255, 0, 0);
	public static final Color RED_DIMMED = new Color(120, 0, 0);
	public static final Color PINK = new Color(255, 18, 143);
	public static final Color GREEN = new Color(0, 255, 8);
	public static final Color GREEN_DIMMED = new Color(0, 50, 0);
	public static final Color PURPLE = new Color(196, 18, 255);
	public static final Color ORANGE = new Color(255, 53, 13);
	public static final Color YELLOW = new Color(255, 150, 5);
	public static final Color CYAN = new Color(52, 155, 235);
	public static final Color BLUE = new Color(0, 0, 255);
	public static final Color BLUE_DIMMED = new Color(0, 0, 120);
  
  public static Color ledOff() {
		return new Color(0, 0, 0);
	}

  public LedState(int r, int g, int b) {
    blue = b;
    green = g;
     red = r;
  }

  public enum AnimationState{
    LARSON,
    RAINBOW,
    FIRE,
    SOLID,
    TWINKLE,
    FADE,
    RGBFADE,
    STROBE
  }
}