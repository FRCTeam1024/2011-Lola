package org.mckenzierobotics.y2011.testrobot.hardware;

import com.sun.squawk.util.MathUtils;

/**
 *
 * @author LukeShu
 */
public class IRSensorLong extends IRSensor {
	public static final double CONV_SCALAR = 58.43;
	public static final double CONV_EXPONENT = -1.249;

	public IRSensorLong(int channel) {
		super(channel);
	}

	public IRSensorLong(int slot, int channel) {
		super(slot,channel);
	}

	public static double voltageToDistance(double v) {
		return CONV_SCALAR*MathUtils.pow(v, CONV_EXPONENT);
	}
}
