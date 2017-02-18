package org.mckenzierobotics.y2011.testrobot.hardware;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

/**
 *
 * @author LukeShu
 */
public class PIDController extends edu.wpi.first.wpilibj.PIDController implements PIDOutput {
	public PIDController(double Kp, double Ki, double Kd, PIDSource source, PIDOutput output) {
		super(Kp, Ki, Kd, source, output);
	}
	public PIDController(double Kp, double Ki, double Kd, PIDSource source, PIDOutput output, double period) {
		super(Kp, Ki, Kd, source, output, period);
	}
	public void pidWrite(double output) {
		setSetpoint(output);
	}
}
