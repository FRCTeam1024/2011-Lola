package org.mckenzierobotics.y2011.testrobot.hardware;

import org.mckenzierobotics.y2011.testrobot.Func;
import edu.wpi.first.wpilibj.Solenoid;

/**
 *
 * @author LukeShu
 */
public class ElbowPneumaticDual implements Elbow {
	private int position;
	private final Solenoid solSide;
	private final Solenoid solHeight;

	public ElbowPneumaticDual(Solenoid solSide, Solenoid solHeight) {
		this.solSide = solSide;
		this.solHeight = solHeight;
	}

	public boolean isInPosition() {
		return true;
	}
	public int get() {
		return position;
	}

	/* The relationship between position and the value of the solenoids:
	 * (assuming true is extended)
	 *
	 * (solSide?(1<<1):0)+(solHeight?(1<<0):0) = position
	 * (solSide?2:0)+(solHeight?1:0) = position
	 *
	 */

	public void set(int pos) {
		position = pos;
		// assuming false is retracted
		boolean solSide_val   = (position&(1<<1))!=0;
		boolean solHeight_val = (position&(1<<0))!=0;

		  solSide.set(!solSide_val);
		solHeight.set( solHeight_val);
	}

	private int oldDiff;
	public void shift(int diff) {
		if (diff!=oldDiff) {
			position += diff;
			position = Math.max(0, position);
			position = Math.min(3, position);
			set(position);
		}
		oldDiff = diff;
	}
	
	public void disable() {}
	public void enable() {}
}
