package org.mckenzierobotics.y2011.testrobot.hardware;

import edu.wpi.first.wpilibj.AnalogChannel;

/**
 *
 * @author LukeShu
 */
public class IRSensor extends AnalogChannel {
	public IRSensor(int channel) {
		super(channel);
	}

	public IRSensor(int slot, int channel) {
		super(slot,channel);
	}

	public static double voltageToDistance(double v) {
		return v;
	}

	public double getDistance() {
		return voltageToDistance(getVoltage());
	}

	public double pidGet() {
		return getDistance();
	}
}
