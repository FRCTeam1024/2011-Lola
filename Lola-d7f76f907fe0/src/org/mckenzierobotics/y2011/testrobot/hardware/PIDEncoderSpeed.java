package org.mckenzierobotics.y2011.testrobot.hardware;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Encoder;

/**
 *
 * @author LukeShu
 */
public class PIDEncoderSpeed implements PIDSource {
	private Encoder enc;
	public PIDEncoderSpeed(Encoder encoder) { enc = encoder; }
	public double pidGet() { return enc.getRate(); }
}
