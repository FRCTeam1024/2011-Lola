package org.mckenzierobotics.y2011.testrobot;

import edu.wpi.first.wpilibj.Encoder;
import org.mckenzierobotics.y2011.testrobot.hardware.LCD;
import org.mckenzierobotics.y2011.testrobot.hardware.Arm;
import org.mckenzierobotics.y2011.testrobot.hardware.PIDController;

/**
 *
 * @author LukeShu
 */
public class Control {
	// Robot to Controller /////////////////////////////////////////////////////

	public static boolean armInPosition = false;
	public static Encoder lEncoder;
	public static Encoder rEncoder;
	
	// Controller to Robot /////////////////////////////////////////////////////

	public static double lPower = 0;
	public static double rPower = 0;
	public static PIDController lSpeed;
	public static PIDController rSpeed;
	/** 0=auto 1=low 2=high */public static int shift = 0;

	/** arm mode, null=manual */public static Arm.Mode armMode = null;
	/** in self control mode  */public static boolean armFront = false;
	/** in center column?     */public static boolean armMiddle = false;

	/** for armMode=null */public static double armShoulder = 0;
	/** for armMode=null */public static int armElbow = 0;

	public static boolean backFingerOpen = false;
	public static boolean frontFingerOpen = false;
	
	public static boolean calibrate = false;
	public static boolean calibrate0 = false;

	public static boolean minibotArmOut = false;
	public static boolean minibotRelease = false;
	
	// Back and Forth //////////////////////////////////////////////////////////

	public static final LCD lcd = new LCD();
}
