package org.mckenzierobotics.y2011.testrobot.hardware;

/**
 *
 * @author LukeShu
 */
public interface Elbow {
	public boolean isInPosition();
	public int get();
	/**
	 *   [1] [2]
	 *     \ /
	 * [0]--*--[3]
	 * Back  Front
	 *
	 * @param pos
	 */
	public void set(int pos);
	public void shift(int diff);
	public void disable();
	public void enable();
}
