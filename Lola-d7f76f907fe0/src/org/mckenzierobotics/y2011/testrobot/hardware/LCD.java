package org.mckenzierobotics.y2011.testrobot.hardware;

import edu.wpi.first.wpilibj.DriverStationLCD;

/**
 * High-er level driver for the Driver Station LCD.
 *
 * @author LukeShu, michandrz
 */
public class LCD {
	private DriverStationLCD driverst = DriverStationLCD.getInstance();

	/**
	 * The contents of the 6 lines, each index of the array is an LCD line.
	 * You can access these directly, just call update() when you are done.
	 *
	 * @see #update()
	 */public String lines[]={"0:","1:","2:","3:","4:","5:"};
	
	private static final DriverStationLCD.Line lineKeys[] = {
		DriverStationLCD.Line.kMain6,
		DriverStationLCD.Line.kUser2,
		DriverStationLCD.Line.kUser3,
		DriverStationLCD.Line.kUser4,
		DriverStationLCD.Line.kUser5,
		DriverStationLCD.Line.kUser6,
	};

	private String spaces;

	/**
	 * Initialize the LCD class
	 */
	public LCD() {
		// create a string full of spaces, the length of an LCD row.
		char spaceA[]=new char[DriverStationLCD.kLineLength];
		for (int i=0;i<DriverStationLCD.kLineLength;i++) {
			spaceA[i]=' ';
		}
		spaces = new String(spaceA);
	}

	/**
	 * Print a new line, and scroll the old lines.
	 * This calls update() for you.
	 *
	 * @see #update()
	 * @param line
	 */
	public void println(String line) {
		for (int i=0;i<(lines.length-1);i++)
			lines[i]=lines[i+1];
		lines[lines.length-1]=line;
		update();
	}

	/**
	 * updated the screen based on the lines[] variable.
	 *
	 * @see #lines
	 */
	public void update() {
		for (int i=0;i<(lines.length);i++) {
			if (lines[i].length()<DriverStationLCD.kLineLength) {
				lines[i]=lines[i]+spaces.substring(lines[i].length());
			}
			driverst.println(lineKeys[i], 1, lines[i]);
		}
		driverst.updateLCD();
	}
	
}
