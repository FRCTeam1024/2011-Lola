package org.mckenzierobotics.y2011.testrobot.hardware;

import com.sun.squawk.util.MathUtils;

/**
 *
 * @author LukeShu
 */
public class IRSensorMid extends IRSensor {
	public static final double CONV_SCALAR = 28.954;
	public static final double CONV_EXPONENT = -1.3044;

	public IRSensorMid(int channel) {
		super(channel);
	}

	public IRSensorMid(int slot, int channel) {
		super(slot,channel);
	}

	public static double voltageToDistance(double v) {
		return CONV_SCALAR*MathUtils.pow(v, CONV_EXPONENT);
	}
}
