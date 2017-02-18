package org.mckenzierobotics.y2011.testrobot.hardware;

import edu.wpi.first.wpilibj.PIDSource;

/**
 *
 * @author LukeShu
 */
public interface AbsoluteEncoder extends PIDSource {
	public void setZeroDegrees(double d);
	public void setZeroRadians(double r);
	public void setZeroVoltage(double v);
	public double getDegrees();
	public double getRadians();

	public void rotateZeroDegrees(double d);
	public void rotateZeroRadians(double r);
	public void rotateZeroVoltage(double v);

	public void calibrateDegrees(double d);
	public void calibrateRadians(double r);
}
