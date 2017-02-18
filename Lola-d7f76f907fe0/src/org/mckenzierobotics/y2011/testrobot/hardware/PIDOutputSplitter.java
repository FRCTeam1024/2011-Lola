package org.mckenzierobotics.y2011.testrobot.hardware;

import edu.wpi.first.wpilibj.PIDOutput;
import org.mckenzierobotics.y2011.testrobot.Debug;

/**
 *
 * @author LukeShu
 */
public class PIDOutputSplitter implements PIDOutput {
	private final PIDOutput[] outputs;
	private final double[] scalars;

	public PIDOutputSplitter(PIDOutput[] outputs, double[] scalars) {
		this.outputs = outputs;
		this.scalars = scalars;
		if (outputs.length != scalars.length) {
			throw new RuntimeException("outputs and scalars must be same len");
		}
	}

	public PIDOutputSplitter(PIDOutput[] outputs) {
		double[] s = new double[outputs.length];
		for (int i=0; i<s.length; i++) {
			s[i]=1;
		}
		this.outputs = outputs;
		this.scalars = s;
	}

	public void pidWrite(double output) {
		for(int i=0; i<outputs.length; i++) {
			try {
				outputs[i].pidWrite(output*scalars[i]);
			} catch (Exception e) {
				Debug.err(e);
			}
		}
	}
}
