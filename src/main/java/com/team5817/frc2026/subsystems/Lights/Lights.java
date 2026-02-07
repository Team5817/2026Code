package com.team5817.frc2026.subsystems.Lights;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Enum representing different LED states with associated colors and intervals.
 */
public enum Lights {
	OFF("OFF", LightsConstants.off()),
	IDLE("IDLE", LightsConstants.CYAN);


	/**
	 * Array of colors to iterate over.
	 */
	public final Color[] colors;

	/**
	 * Time in seconds between states.
	 */
	public final double interval;

	/**
	 * Name of the state.
	 */
	public final String name;

	/**
	 * Constructor for states with a specified interval.
	 *
	 * @param name the name of the state
	 * @param interval the time interval in seconds
	 * @param colors the colors associated with the state
	 */
	Lights(String name, double interval, Color... colors) {
		this.colors = colors;
		this.interval = interval;
		this.name = name;
	}

	/**
	 * Constructor for states with an infinite interval.
	 *
	 * @param name the name of the state
	 * @param colors the colors associated with the state
	 */
	TimedLEDState(String name, Color... colors) {
		this.colors = colors;
		this.interval = Double.POSITIVE_INFINITY;
		this.name = name;
	}

	/**
	 * Gets the colors associated with the state.
	 *
	 * @return the colors
	 */
	public Color[] getColors() {
		return colors;
	}

	/**
	 * Gets the interval time in seconds.
	 *
	 * @return the interval
	 */
	public double getInterval() {
		return interval;
	}

	/**
	 * Gets the name of the state.
	 *
	 * @return the name
	 */
	public String getName() {
		return name;
	}
}