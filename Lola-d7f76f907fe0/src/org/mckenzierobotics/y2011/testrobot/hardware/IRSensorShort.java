package org.mckenzierobotics.y2011.testrobot.hardware;

import com.sun.squawk.util.MathUtils;

/**
 *
 * @author LukeShu
 */
public class IRSensorShort extends IRSensor {
	public static final double CONV_SCALAR = 13.345;
	public static final double CONV_EXPONENT = -1.2096;

	public IRSensorShort(int channel) {
		super(channel);
	}

	public IRSensorShort(int slot, int channel) {
		super(slot,channel);
	}

	public static double voltageToDistance(double v) {
		return CONV_SCALAR*MathUtils.pow(v, CONV_EXPONENT);
	}
}
