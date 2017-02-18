package org.mckenzierobotics.y2011.testrobot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDOutput;

/**
 *
 * @author LukeShu
 */
public class RollingAvg implements PIDSource, PIDOutput {
	private PIDSource source = null;
	private double[] points;
	private double avg;
	private int i;

	public RollingAvg(int len) {
		points = new double[len];
		i = 0;
		avg = 0;
	}
	public RollingAvg(int len, PIDSource src) {
		this(len);
		source = src;
	}

	public double push(double v) {
		avg -= points[i];
		points[i] = v/points.length;
		avg += points[i];
		i++; i %= points.length;
		return avg;
	}

	public double get() {
		return avg;
	}

	public double pidGet() {
		if (source!=null) return push(source.pidGet());
		return get();
	}
	
	public void pidWrite(double output) {
		push(output);
	}
}
