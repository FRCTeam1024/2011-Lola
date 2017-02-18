package org.mckenzierobotics.y2011.testrobot.hardware;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

/**
 *
 * @author LukeShu
 */
public class PIDValue implements PIDOutput, PIDSource {
	public double value = 0;
	public void pidWrite(double output) { value = output; }
	public double pidGet() { return value; }
}
