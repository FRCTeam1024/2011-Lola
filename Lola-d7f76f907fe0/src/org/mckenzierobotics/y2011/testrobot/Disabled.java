package org.mckenzierobotics.y2011.testrobot;

/**
 *
 * @author LukeShu
 */
public class Disabled implements Controller {
	public void init() {

	}

	public void periodic() {
		Control.lPower = 0;
		Control.rPower = 0;
		Control.minibotRelease = false;
	}
}
