package org.mckenzierobotics.y2011.testrobot.hardware;

import edu.wpi.first.wpilibj.AnalogChannel;
import org.mckenzierobotics.y2011.testrobot.Debug;

/**
 *
 * @author LukeShu
 */
public class MagneticEncoder extends AnalogChannel implements AbsoluteEncoder {
	public static final double MAX_VOLTS = 5.0;
	public static final double voltsToDegrees = 360/MAX_VOLTS;
	public static final double voltsToRadians = (2*Math.PI)/MAX_VOLTS;

	double zeroVolts = 0;

	public MagneticEncoder(int channel) {
		super(channel);
	}

	public MagneticEncoder(int channel, double zeroVolts) {
		super(channel);
		setZeroVoltage(zeroVolts);
	}

	public MagneticEncoder(int slot, int channel) {
		super(slot,channel);
	}

	public MagneticEncoder(int slot, int channel, double zeroVolts) {
		super(slot,channel);
		setZeroVoltage(zeroVolts);
	}

	public void setZeroDegrees(double d) { setZeroVoltage(d/voltsToDegrees); }
	public void setZeroRadians(double r) { setZeroVoltage(r/voltsToRadians); }
	public void setZeroVoltage(double v) { zeroVolts = v; }

	public void rotateZeroDegrees(double d) { rotateZeroVoltage(d/voltsToDegrees); }
	public void rotateZeroRadians(double r) { rotateZeroVoltage(r/voltsToRadians); }
	public void rotateZeroVoltage(double v) {
		double newZeroVolts = zeroVolts+v;
		while (newZeroVolts < 0) {
			newZeroVolts += MAX_VOLTS;
		}
		if (newZeroVolts > MAX_VOLTS) {
			newZeroVolts %= MAX_VOLTS;
		}
		setZeroVoltage(newZeroVolts);
	}

	public double getDegrees() { return getVoltage()*voltsToDegrees; }
	public double getRadians() { return getVoltage()*voltsToRadians; }
	public double getVoltage() {
		double v =getRawVoltage()-zeroVolts;
		if (v < -(MAX_VOLTS/2)) { v = ( MAX_VOLTS+v); }
		if (v >  (MAX_VOLTS/2)) { v = (-MAX_VOLTS+v); }
		return v;
	}

	public void calibrateDegrees(double d) {
		setZeroVoltage(getRawVoltage());
		rotateZeroDegrees(-d);
	}
	public void calibrateRadians(double r) {
		setZeroVoltage(getRawVoltage());
		rotateZeroRadians(-r);
	}

	public double getRawVoltage() {
		return super.getVoltage();
	}

	public double pidGet() { return getDegrees(); }
}
