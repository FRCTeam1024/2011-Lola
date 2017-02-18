package org.mckenzierobotics.y2011.testrobot;

import org.mckenzierobotics.y2011.testrobot.hardware.PIDController;
import org.mckenzierobotics.y2011.testrobot.hardware.PIDEncoderDistance;
import edu.wpi.first.wpilibj.Timer;

/**
 * RALFF - Robotics Autonomous Language For FIRST
 *
 * This gives you very general commands to use when creating autonomous.  Look
 * in Autonomous.java for examples of how to use this.
 *
 * So, it's not actually a language, because I never got around to writing an
 * interpreter (extra lame since other teams actually have interpreted
 * languages that they wrote).  But it works well enough.
 *
 * HACKING:
 * To add a new command, create a public boolean method, with any desired
 * parameters.  If it is done doing it's thing, return true, so we can go on to
 * the next step.  If it is not done doing its thing, and needs to run another
 * iteration, then return false.  You may use the class variable ``tick'' to
 * store a boolean value, however ALWAYS reset it to false before returning
 * true.  Reftime is when the previous command ended/this one started, so also
 * set `refTime=timer.get();' when returning true.
 *
 * @author LukeShu, LOLCAT
 */
public class RALFF {
	private boolean tick = false;
	private double d = 0;
	private PIDController lDistPID;
	private PIDController rDistPID;
	public Timer timer = new Timer();
	private double refTime = 0;

	private static double lScale = 1.00;//0.95;
	private static double rScale = 1.00;

	/*<editor-fold desc="RALFF generic">***************************************\
	 *                          RALFF generic                                 *
	\**************************************************************************/

	/**
	 * Start/reset the timers and such.  This should be run immediately before
	 * any other RALFF commands.
	 *
	 * @return go to next step?
	 */
	public boolean init() {
		tick = false;
		timer.start();
		refTime = timer.get();

		lDistPID = new PIDController(1,0,0,
				new PIDEncoderDistance(Control.lEncoder),
				Control.lSpeed);
		rDistPID = new PIDController(1,0,0,
				new PIDEncoderDistance(Control.rEncoder),
				Control.rSpeed);

		lDistPID.disable();
		rDistPID.disable();

		return true;
	}

	/**
	 * Delay for a given amount of time
	 * @param time time to delay in seconds
	 * @return go to next step?
	 */
	public boolean sleep(double time) {
		if (timer.get()<refTime+time) {
			return false;
		} else {
			refTime = timer.get();
			return true;
		}
	}

	//</editor-fold>

	/*<editor-fold desc="Lola misc.">******************************************\
	 *                          Lola miscellaneous                            *
	\**************************************************************************/


	public boolean setForward(boolean forward) {
		Control.armFront = forward;
		refTime = timer.get();
		return true;
	}

	public boolean setHand(boolean frontOpen, boolean backOpen) {
		Control.frontFingerOpen = frontOpen;
		Control.backFingerOpen = backOpen;
		refTime=timer.get();
		return true;
	}

	/* 0=auto 1=low 2=high */
	public boolean shiftAuto() { return shift(0); }
	public boolean shiftLow( ) { return shift(1); }
	public boolean shiftHigh() { return shift(2); }
	public boolean shift(int s) {
		Control.shift = s;
		refTime=timer.get();
		return true;
	}

	//</editor-fold>

	/*<editor-fold desc="Arm">*************************************************\
	 *                               Arm                                      *
	\**************************************************************************/

	public boolean calibrateArm() {
		if (!tick) {
			Control.calibrate = true;
			tick = true;
			return false;
		} else {
			tick = false;
			Control.calibrate = false;
			refTime=timer.get();
			return true;
		}
	}

	public boolean moveArm(org.mckenzierobotics.y2011.testrobot.hardware.Arm.Mode mode, boolean wait) {
		Control.armMode = mode;
		if (!tick) {
			tick = wait;
			return !wait;
		}
		if (Control.armInPosition) {
			tick = false;
			refTime=timer.get();
			return true;
		}
		return false;
	}

	//</editor-fold>

	/*<editor-fold desc="Drive">***********************************************\
	 *                                Drive                                   *
	\**************************************************************************/

	public boolean driveTime(double lSpeed, double rSpeed, double time) {
		Control.lPower = lSpeed;
		Control.rPower = rSpeed;
		if ((timer.get()-refTime)>=time) {
			Control.lPower = Control.rPower = 0;
			refTime=timer.get();
			return true;
		}
		return false;
	}

	public boolean driveDistancePower(double lDist, double rDist, double power) {
		return driveDistancePower(lDist, rDist, power, power);
	}
	public boolean driveDistancePower(double lDist, double rDist, double lPower, double rPower) {
		if (!tick) {
			Control.lEncoder.reset();
			Control.rEncoder.reset();
			tick = true;
		}
		double lDiff = lDist - Control.lEncoder.getDistance();
		double rDiff = rDist - Control.rEncoder.getDistance();
		if ( (Math.abs(lDiff)<.1) && (Math.abs(rDiff)<.1) ) {
			Control.lPower = Control.rPower = 0;
			tick = false;
			refTime=timer.get();
			return true;
		}
		double lRatio = 1;
		double rRatio = 1;
		if ((lDiff/rDiff)>1.0) {
			// left is larger
			rRatio = (Math.abs(rDiff)/Math.abs(lDiff));
		} else
		if ((rDiff/lDiff)>1.0) {
			// right is larger
			lRatio = (Math.abs(lDiff)/Math.abs(rDiff));
		}
		Control.lPower = lPower*Func.sign(lDiff)*lScale*lRatio;
		Control.rPower = rPower*Func.sign(rDiff)*rScale*rRatio;
		return false;
	}

	public boolean driveSpeed(double lSpeed, double rSpeed, double time) {
		if (!tick) {
			Control.lSpeed.enable();
			Control.rSpeed.enable();
			Control.lSpeed.setSetpoint(lSpeed);
			Control.rSpeed.setSetpoint(rSpeed);
			tick = true;
		}

		if ((timer.get()-refTime)>=time) {
			Control.lSpeed.disable();
			Control.rSpeed.disable();

			Control.lPower = Control.rPower = 0;
			tick = false;
			refTime=timer.get();
			return true;
		}
		return false;
	}

	public boolean drivePID(double lDist, double rDist, double speed) {
		//<editor-fold desc="init">/////////////////////////////////////////////
		if (!tick) {
			Control.lEncoder.reset();
			Control.rEncoder.reset();

			lDistPID.setOutputRange(-speed, speed);
			rDistPID.setOutputRange(-speed, speed);
			lDistPID.enable();
			rDistPID.enable();

			Control.lSpeed.setSetpoint(0);
			Control.rSpeed.setSetpoint(0);
			Control.lSpeed.enable();
			Control.rSpeed.enable();

			tick = true;
		}
		//</editor-fold>

		//<editor-fold desc="periodic">/////////////////////////////////////////
		double lPct = Control.lEncoder.getDistance()/lDist;
		double rPct = Control.rEncoder.getDistance()/rDist;

		if (Math.abs(lPct-rPct)>0.02) {
			// Left and right are too far out of sync, correct!
			if (d==0) d = Math.max(lPct,rPct);
			double dist = Math.max(lDist, rDist);

			lDistPID.setInputRange(-3*d*dist, 3*d*dist);
			rDistPID.setInputRange(-3*d*dist, 3*d*dist);

			lDistPID.setPID(.2,0,.7);
			rDistPID.setPID(.2,0,.7);

			lDistPID.setSetpoint(d*lDist);
			rDistPID.setSetpoint(d*rDist);
		} else {
			// Keep on keeping on.
			d = 0;

			lDistPID.setInputRange(-2*lDist, 2*lDist);
			rDistPID.setInputRange(-2*rDist, 2*rDist);

			lDistPID.setPID(1,0,0);
			rDistPID.setPID(1,0,0);

			lDistPID.setSetpoint(lDist);
			rDistPID.setSetpoint(rDist);
		}
		//</editor-fold>

		//<editor-fold desc="finish">///////////////////////////////////////////
		double lDiff = lDist - Control.lEncoder.getDistance();
		double rDiff = rDist - Control.rEncoder.getDistance();
		if ( (Math.abs(lDiff)<.1) &&
		     (Math.abs(rDiff)<.1) ){
			lDistPID.disable();
			rDistPID.disable();
			Control.lSpeed.disable();
			Control.rSpeed.disable();

			Control.lPower = Control.rPower = 0;
			tick = false;
			refTime=timer.get();
			return true;
		}
		//</editor-fold>
		return false;
	}

	//</editor-fold>
}
