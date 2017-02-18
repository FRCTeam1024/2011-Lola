package org.mckenzierobotics.y2011.testrobot.hardware;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDController;
import com.sun.squawk.util.MathUtils;
import org.mckenzierobotics.y2011.testrobot.Func;

/**
 *
 * @author LukeShu
 */
public class ElbowPID implements Elbow {
	private static final double[] angles  = { 20, 95 };// TODO: tune
	private static final double[] positions = {
		-angles[1],
		-angles[0],
		 angles[0],
		 angles[1]
	};
	private static final double[] PID = { 0.02, 3, 0};
	private static final double tolerance = 5;

	private final AbsoluteEncoder e;
	private final PIDOutput       m;
	private final PIDController   p;

	private int pos;

	public ElbowPID(AbsoluteEncoder e_elbow, PIDOutput m_elbow) {
		this.e = e_elbow;
		this.m = m_elbow;
		p = new PIDController(PID[0],PID[1],PID[2], e_elbow, m_elbow);

		if (p.isEnable()) { p.disable(); }
		p.setInputRange(-180, 180);
		p.setOutputRange(-.5, .5);
		p.setContinuous(false);
	}

	public int get() {
		double degrees;
		/* Degrees:
		 *       [0]
		 * [-90]--+--[90]
		 */
		degrees = e.pidGet();
		return Func.nearest(degrees, positions);
	}

	public boolean isInPosition() {
		return Math.abs(e.pidGet()-positions[pos])<tolerance;
	}

	public void set(int pos) {
		this.pos = pos;
		p.setSetpoint(positions[pos]);
	}

	public void shift(int diff) {
		if (   p.isEnable()) { p.disable(); }
		m.pidWrite(diff*0.3);
	}

	public void disable() { p.disable(); }
	public void enable() { p.enable(); }
}
