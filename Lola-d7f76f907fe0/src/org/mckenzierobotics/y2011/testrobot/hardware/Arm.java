package org.mckenzierobotics.y2011.testrobot.hardware;

import org.mckenzierobotics.y2011.testrobot.Func;
import org.mckenzierobotics.y2011.testrobot.Debug;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDController;
import com.sun.squawk.util.MathUtils;

/**
 *
 * All angles are in degrees.  All distances are in inches.
 *
 * @author LukeShu, Nathan
 */
public class Arm {

	/**
	 * modes of the arm position
	 */
	public static class Mode {
		public final int val;
		public final double shoulder;
		public final int elbow;
		public final double middleOffset;
		
		protected static final int kLoadFloor_val = 0;
		protected static final int kLoadHuman_val = 1;
		protected static final int kTransport_val = 2;
		protected static final int kBottom_val    = 3;
		protected static final int kMiddle_val    = 4;
		protected static final int kTop_val       = 5;
		
		public static final Mode kLoadFloor = new Mode(kLoadFloor_val);
		public static final Mode kLoadHuman = new Mode(kLoadHuman_val);
		public static final Mode kTransport = new Mode(kTransport_val);
		public static final Mode kBottom    = new Mode(kBottom_val);
		public static final Mode kMiddle    = new Mode(kMiddle_val);
		public static final Mode kTop       = new Mode(kTop_val);
		
		final String[] strings =
			{ "Load - floor", "Load - human", "Transport",
			"Bottom", "Middle", "Top"};

		private Mode(int val) {
			this.val = val;
			/*
			 *   [1] [2]
			 *     \ /
			 * [0]--*--[3]
			 * Back  Front
			 */
			//                         floo,huma,tran,botto,mid, top
			double shoulder_a[]     = { 165, 170, 170, 160, 108,  27 };
			double middleOffset_a[] = {   0,   0,   0,   0, -18, -10};
			int elbow_a[]           = {   3,   2,   1,   2,   2,   2};
			shoulder     =     shoulder_a[val];
			elbow        =        elbow_a[val];
			middleOffset = middleOffset_a[val];
		}

		public String toString() {
			return strings[val];
		}
	}

	/*========================================================================*\
	 *                         Constants (here)                               *
	\*========================================================================*/
	// Physical sizes
	private static final double A = 39; /**< Length of arm nearest the chasis */
	private static final double a = 26; /**< Length of arm farthest from the chasis */
	private static final double b = 47.5; /**< Height of the shoulder off the ground */
	private static final double r = 84-(18.5+2.5); /**< maximum horizontal distance from shoulder */

	// PID constants
	private static final double[] shoulderPID = { 0.05, 1, 0}; // TODO: tune
	
	double shoulderUp = .5;
	double shoulderDown = .3;
	double shoulderBase = (shoulderUp+shoulderDown)/2;
	double shoulderTweak = shoulderUp-shoulderBase;

	// Other
	private static final double tolerance = 2.5;
	private static final double shoulderScalingThreshold = 15;

	/*========================================================================*\
	 *                        Constants (later)                               *
	 *========================================================================*
	 * These are defined during Arm()                                         *
	\*========================================================================*/
	private final AbsoluteEncoder e_shoulder; /**< shoulder encoder */
	private final PIDOutput       m_shoulder; /**< shoulder motor */
	private final PIDController   p_shoulder; /**< shoulder PID controller */
	private final Elbow elbow;
	
	/** shoulder angle required to flip the elbow */
	private final double shoulderFlip;
	/*========================================================================*\
	 *                          Instance Variables                            *
	\*========================================================================*/
	private double shoulderCurrent;
	private double shoulderFinal;

	private int elbowCurrent;
	private int elbowFinal;

	boolean shoulderInFinalPosition = false;
	boolean elbowInFinalPosition = false;
	
	private int side = 1;
	private int sideCurrent = 1;
	private int sideFinal = 1;
	
	private Arm.Mode mode = null;
	private Arm.Mode oldMode = null;

	private boolean middle = false;
	private boolean oldMiddle = false;
	
	/*========================================================================*\
	 *                               Actual Code                              *
	\*========================================================================*/

	public Arm(AbsoluteEncoder e_shoulder, PIDOutput m_shoulder, Elbow elbow) {
		this.e_shoulder = e_shoulder;
		this.m_shoulder = m_shoulder;
		this.elbow = elbow;

		shoulderFlip = MathUtils.asin((r-a)/A)*(180/Math.PI);
		Debug.println("shoulderFlip: "+shoulderFlip);

		p_shoulder = new PIDController(shoulderPID[0],shoulderPID[1],shoulderPID[2], e_shoulder, m_shoulder);
		
		if (p_shoulder.isEnable()) { p_shoulder.disable(); }
		p_shoulder.setInputRange(-180, 180);
		p_shoulder.setContinuous(false);
	}

	public void periodic(Arm.Mode modeArg, boolean middleArg, int sideArg,
			double shoulderManual, int elbowManual) {
		if (modeArg == null) {
			// relax, or manual adjustment
			periodicManual(shoulderManual, elbowManual);
		} else {
			// go to a mode based on sensors
			periodicPosition(modeArg, middleArg, sideArg);
		}
		oldMode = mode;
		oldMiddle = middle;
	}

	private void periodicManual(double shoulderManual, int elbowManual) {
		if (p_shoulder.isEnable()) { p_shoulder.disable(); }
		m_shoulder.pidWrite(shoulderManual);
		elbow.shift(elbowManual);
		shoulderInFinalPosition = elbowInFinalPosition = true;
	}

	public boolean isInPosition() {
		return shoulderInFinalPosition && elbowInFinalPosition;
	}

	public double flipShoulder(double shoulder) {
		return shoulder*-1;
	}

	public int flipElbow(int elbow) {
		return 3-elbow;
	}

	private void periodicPosition(Arm.Mode modeArg, boolean middleArg, int sideArg) {
		mode = modeArg;
		middle = middleArg;

		// Calculate Final Targets /////////////////////////////////////////////
		shoulderFinal = mode.shoulder+(middle?mode.middleOffset:0);
		elbowFinal = mode.elbow;
		sideFinal = sideArg;
		if (sideFinal == -1) {
			shoulderFinal = flipShoulder(shoulderFinal);
			elbowFinal = flipElbow(elbowFinal);
		}

		// Update Sensors //////////////////////////////////////////////////////
		shoulderCurrent = e_shoulder.pidGet();
		elbowCurrent   = elbow.get();
		sideCurrent = Func.sign(shoulderCurrent);
		
		// Caclulate setpoints /////////////////////////////////////////////////
		double shoulderSetpoint = shoulderFinal; // the shoulder's easy
		int elbowSetpoint;// the elbow... not so much:

		double shoulderCH = shoulderCurrent-90;// shoulder current horiz
		double shoulderFH = shoulderFinal-90;// shoulder final horiz

		boolean sameZone = Func.sign(shoulderCH) == Func.sign(shoulderFH);
		boolean flatOK = Math.abs(shoulderCH) > (90-shoulderFlip);
		if (sameZone && flatOK) {
			elbowSetpoint = elbowFinal;
		} else {
			elbowSetpoint = (int)Func.nearest(elbowFinal, 1,2);
		}

		// Scale Output Ranges /////////////////////////////////////////////////
		if (Math.abs(shoulderCurrent)>Math.abs(shoulderScalingThreshold)) {
			p_shoulder.setOutputRange(
				-shoulderBase-(sideCurrent*shoulderTweak),
				 shoulderBase-(sideCurrent*shoulderTweak)
			);
		} else {
			p_shoulder.setOutputRange(
				-(shoulderBase+shoulderTweak),
				 (shoulderBase+shoulderTweak)
			);
		}

		// Do motor stuff //////////////////////////////////////////////////////
		if (shoulderInFinalPosition) {
			p_shoulder.disable();
		} else {
			p_shoulder.enable();
			p_shoulder.setSetpoint(shoulderSetpoint);
		}
		elbow.set(elbowSetpoint);
		
		// Update isInPosition() ///////////////////////////////////////////////
		double shoulderFinalDiff = shoulderFinal-shoulderCurrent;
		shoulderInFinalPosition = Math.abs(shoulderFinalDiff) < tolerance;
		elbowInFinalPosition = elbow.isInPosition() && (elbowSetpoint == elbowFinal);
	}

}
