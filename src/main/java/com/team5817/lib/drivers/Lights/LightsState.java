package com.team5817.lib.drivers.Lights;

import edu.wpi.first.wpilibj.util.Color;

public class LightsState {
  public static final Color RED = new Color(255, 0, 0);
  public static final Color OFF = new Color(0, 0, 0);
  public static final Color PINK = new Color(255, 18, 143);
  public static final Color GREEN = new Color(0, 255, 8);
  public static final Color PURPLE = new Color(196, 18, 255);
  public static final Color ORANGE = new Color(255, 53, 13);
  public static final Color YELLOW = new Color(255, 150, 5);
  public static final Color CYAN = new Color(52, 155, 235);
  public static final Color BLUE = new Color(0, 0, 255);
  public static final Color WHITE = new Color(255, 255, 255);
  public static final Color[] RAINBOW = {
    WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, WHITE, OFF, RED, ORANGE, YELLOW, GREEN, CYAN,
    BLUE, PURPLE, OFF, OFF
  };

  public enum LEDState {
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
    TWINKLE_WHITE(Color.kWhite, "TWINKLE"),
    RED(Color.kRed, "SOLID"),
    BLUE(Color.kBlue, "SOLID"),
    YELLOW(Color.kYellow, "SOLID"),
    TEAL(Color.kTeal, "SOLID"),
    PURPLE(Color.kPurple, "SOLID"),
    NONE(Color.kWhite, "SOLID"),
    BLINK_BLUE(Color.kBlue, "STROBE"),
    RAINBOW(null, "RAINBOW");

    Color color = null;
    String animation = null;

    private LEDState(Color color, String animation) {
      this.color = color;
      this.animation = animation;
    }
  }
}
