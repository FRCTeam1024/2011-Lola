package org.mckenzierobotics.y2011.testrobot.hardware;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 *
 * @author LukeShu
 */
public class LineSensor {
	private DigitalInput dl, dm, dr;
	private boolean l, m, r;
	private boolean hasLine;

	public LineSensor(DigitalInput left,
	                  DigitalInput middle,
	                  DigitalInput right)
	{
		this(left, middle, right, false);
	}

	public LineSensor(DigitalInput left,
	                  DigitalInput middle,
	                  DigitalInput right,
	                  boolean onLine)
	{
		dl = left;
		dm = middle;
		dr = right;
		this.hasLine = onLine;
		l = m = r = false;
	}

	public void update() {
		l = dl.get();
		m = dm.get();
		r = dr.get();
		hasLine |= m;
	}

	/**
	 * on a scale of -2 through 2, how much do we need to turn/which way?
	 * -2 is hard left, 0 is straight, 2 is hard right.
	 *
	 * @return
	 */
	public int getDirection() {
		return ((r?2:0)-(l?2:0))/(m?2:1);
	}

	public boolean hasLine() {
		return hasLine;
	}

	public boolean atTee() {
		return l && m && r;
	}

	public int getRaw() {
		int data =
			((l?1:0)   )+
			((m?1:0)<<1)+
			((r?1:0)<<2);
		return data;
	}

	public void reset() {
		hasLine = false;
	}
}
