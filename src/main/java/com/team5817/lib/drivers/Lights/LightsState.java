package com.team5817.lib.drivers.Lights;

import edu.wpi.first.wpilibj.util.Color;

public class LightsState {
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
   /* Animation List
     * SOLID
     * LARSON
     * RGBFADE
     * FIRE
     * STROBE
     * RAINBOW
     * FLOW
    */
    ORANGE(Color.kOrange, "SOLID"),
    DUALMODE(Color.kWhite, "SOLID"),
    ALLIANCERED(Color.kRed, "SOLID"),
    ALLIANCEBLUE(Color.kBlue, "SOLID"),
    NOTLOCKED(Color.kYellow, "SOLID"),
    LOCKED(Color.kGreen, "SOLID"),
    CLIMBING(Color.kPurple, "SOLID"),
    NONE(Color.kWhite, "SOLID"),
    HOPPEREMPTY(Color.kOrange, "STROBE");
    private LEDState(){

    }
    Color color = null;
    String animation = null;
    private LEDState(Color color, String animation){
      this.color = color;
      this.animation = animation;
    }
  }
}

/*Idle: White
Not Locked: Yellow
Locked: Green
Dual mode: Fire
Climb: Purple 
Hopper empty: Flash Orange
Alliance shift: Red or Blue */