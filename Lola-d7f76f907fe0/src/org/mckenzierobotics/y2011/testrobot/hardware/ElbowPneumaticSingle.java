package org.mckenzierobotics.y2011.testrobot.hardware;

import org.mckenzierobotics.y2011.testrobot.Func;
import edu.wpi.first.wpilibj.Solenoid;

/**
 *
 * @author LukeShu
 */
public class ElbowPneumaticSingle implements Elbow {
	private int position;
	private final Solenoid sol;

	public ElbowPneumaticSingle(Solenoid sol) {
		this.sol = sol;
	}

	public boolean isInPosition() {
		return (position != 0);
	}
	public int get() {
		return (Math.max(1,position));
	}

	public void set(int pos) {
		position = pos;
		switch (position) {
			case 0:
			case 1: sol.set(false); break;
			case 2: sol.set(false); break;
			case 3: sol.set(true); break;
		}
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
