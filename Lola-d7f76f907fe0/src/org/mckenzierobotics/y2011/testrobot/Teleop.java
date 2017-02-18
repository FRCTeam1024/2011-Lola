package org.mckenzierobotics.y2011.testrobot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO.EnhancedIOException;
import org.mckenzierobotics.y2011.testrobot.hardware.Arm;
import com.sun.squawk.util.MathUtils;

/**
 * Like an AI, but read from the drive station.
 *
 * @author tjroche, LOLCAT, LukeShu, poejreed, Nathan
 */
public class Teleop implements Controller {
	/*<editor-fold desc="Declarations and Instantiations">*****************\
	 *                   Declarations and Instantiations                  *
	\**********************************************************************/

	// Joysticks
	private boolean joystickFix = false;
	private Joystick rJoystick = new Joystick(1);
	private Joystick lJoystick = new Joystick(2);
	private double rJoystickY;
	private double lJoystickY;
	private Timer timer = new Timer();
	// Driver station
	DriverStation Driver;
	DriverStationEnhancedIO eDriver;
	private boolean usingEnhancedIO = true;
	// Button states
	private boolean button_armTop = false;
	private boolean button_armMid = false;
	private boolean button_armBot = false;
	private boolean button_armTrans = false;
	private boolean button_armFloor = false;
	private boolean button_armHuman = false;
	private boolean switch_armSU = false;
	private boolean switch_armSD = false;
	private boolean switch_armEU = false;
	private boolean switch_armED = false;
	private boolean switch_middle = false;
	private boolean switch_armFront = false;
	private boolean button_miniOut = false;
	private boolean button_miniRel = false;

	private double refTime = 0;

	//</editor-fold>
	
	/**
	 * Create a new instance of the Teleoperated controller.
	 */
	public Teleop() {
		init();
	}

	public void init() {
		timer.reset();
		timer.start();
		Driver = DriverStation.getInstance();
		eDriver = Driver.getEnhancedIO();

		Control.lSpeed.disable();
		Control.rSpeed.disable();

		try {
			int counter = 0;
			byte version = 0;
			while (version == 0) {
				version = eDriver.getFirmwareVersion();
				Debug.println("DriverStation FWversion: "+version);
				counter++;
			}
			Debug.println("Tries requiered to get DS FWversion: "+counter);
		} catch (EnhancedIOException e) {
			Debug.println("Enhanced IO Driver station not available.");
			e.printStackTrace();
		}
	}

	private boolean getDigitalIn(int dio) {
		boolean b;
		try {
			b = !eDriver.getDigital(dio);
		} catch (Exception e) {
			b = false;
			e.printStackTrace();
		}
		return b;
	}

	/**
	 * Read from the drive station to update the control structure.
	 */
	public void periodic() {
		/*<editor-fold desc="Update Buttons">**********************************\
		 *                           Update Buttons                           *
		\**********************************************************************/

		/*
		 * Cypress Breakout Board:
		 *
		 * GND      	GND
		 * -------------------------
		 * P2_7  D16	P2_6  D15
		 * P12_3 D14	P12_2 D13
		 * P6_7  D12	P6_6  D11
		 * P6_5  D10	P6_4  D09
		 * P6_3  D08	P6_2  D07
		 * P6_1  D06	P6_0  D05
		 * P4_7  D04	P4_6  D03
		 * P4_5  D02	P4_4  D01
		 * -------------------------
		 * P0_7  AI8	P0_6  AI7
		 * P0_5  AI6	P0_4  AI5
		 * P0_3  AI4	P0_2  AI3
		 * P0_1  AI2	P0_0  AI1
		 * -------------------------
		 * VDDIO    	VDDIO
		 */

		/*
		 * DriverStation key:
		 *
		 * +-------+---------------------------------------------
		 * |       |
		 * |       | (D02)                              (D01)
		 * | (D10) |
		 * |       |   (D04)                          (D03)
		 * | (D12) |
		 * |       |     (D06)    +---+    +---+    (D05)
		 * |       |              |D16|    |D15|
		 * |       |              |D14|    |D13|
		 * |       |              +---+    +---+
		 * +-------+--------------------------------------------
		 *         |        f[D08]t             f[D07]t
		 *         |
		 *         +--------------------------------------------
		 */

		button_armTop = getDigitalIn(2); button_armHuman = getDigitalIn(1);
		button_armMid = getDigitalIn(4); button_armFloor = getDigitalIn(3);
		button_armBot = getDigitalIn(6); button_armTrans = getDigitalIn(5);

		switch_armSU = getDigitalIn(16); switch_armEU = getDigitalIn(15);
		switch_armSD = getDigitalIn(14); switch_armED = getDigitalIn(13);

		switch_middle = !getDigitalIn(8); switch_armFront = getDigitalIn(7);

		button_miniOut = getDigitalIn(10);
		button_miniRel = getDigitalIn(12);

		//</editor-fold>

		/*<editor-fold desc="Fix Joysticks">***********************************\
		 *                         Fix Joysticks                              *
		\**********************************************************************/
		if (rJoystick.getRawButton(7)) {
			if (!joystickFix) {
				Joystick tempJoy = lJoystick;
				lJoystick = rJoystick;
				rJoystick = tempJoy;
				joystickFix = true;
			}
		} else {
			joystickFix = false;
		}
		//</editor-fold>

		/*<editor-fold desc="Control for Arm">*********************************\
		 *                        Control for Arm                             *
		\**********************************************************************/

		Control.calibrate  = lJoystick.getRawButton(6);
		Control.calibrate0 = lJoystick.getRawButton(11);

		if (button_armTop) {
			Control.armMode = Arm.Mode.kTop;
			Control.armFront = !switch_armFront;
		} else if (button_armMid) {
			Control.armMode = Arm.Mode.kMiddle;
			Control.armFront = !switch_armFront;
		} else if (button_armBot) {
			Control.armMode = Arm.Mode.kBottom;
			Control.armFront = !switch_armFront;
		} else if (button_armHuman) {
			Control.armMode = Arm.Mode.kLoadFloor;
			Control.armFront = !switch_armFront;
		} else if (button_armFloor) {
			Control.armMode = Arm.Mode.kLoadFloor;
			Control.armFront = switch_armFront;
		} else if (button_armTrans) {
			Control.armMode = Arm.Mode.kTransport;
			Control.armFront = switch_armFront;
		} else {
			Control.armMode = null;
			Control.armFront = switch_armFront;
		}

		Control.armShoulder = (switch_armSU?1:0)-(switch_armSD?1:0);
		Control.armElbow    = (switch_armEU?1:0)-(switch_armED?1:0);
		if (Control.armShoulder == 0) {
			refTime = timer.get();
		} else {
			double time = timer.get()-refTime;
			Control.armShoulder *= .4-MathUtils.pow(.4,(time*1.2)+1.3);
		}

		Control.armMiddle = switch_middle;

		       if (lJoystick.getTrigger()) {
			Control.backFingerOpen = false;
		} else if (lJoystick.getRawButton(3)) {
			Control.backFingerOpen = true;
		}

		       if (rJoystick.getTrigger()) {
			Control.frontFingerOpen = false;
		} else if (rJoystick.getRawButton(3)) {
			Control.frontFingerOpen = true;
		}

		//</editor-fold>

		/*<editor-fold desc="Control for Drive System">************************\
		 *                      Control for Drive System                      *
		\**********************************************************************/
		// joysticks are crazy and return opposite of what you would expect
		rJoystickY = -rJoystick.getY();
		lJoystickY = -lJoystick.getY();
		Control.lPower = lJoystickY*Math.abs(lJoystickY);
		Control.rPower = rJoystickY*Math.abs(rJoystickY);

		if (lJoystick.getRawButton(5)
		  ||rJoystick.getRawButton(5)) {
			Control.shift = 2;
		} else
		if (lJoystick.getRawButton(4)
		  ||rJoystick.getRawButton(4)) {
			Control.shift = 1;
		} else {
			Control.shift = 0;
		}
		//</editor-fold>

		/*<editor-fold desc="Control for Minibot">*****************************\
		 *                       Control for Minibot                          *
		\**********************************************************************/
		Control.minibotArmOut = button_miniOut;
		if (Control.minibotArmOut) {
			Control.minibotRelease |= button_miniRel;
		}
		//</editor-fold>

		/*<editor-fold desc="DriverStationLCD">********************************\
		 *                         DriverStationLCD                           *
		\**********************************************************************/

		/*====================================================================*\
		||         NOTE: USE `Control.whatever' VARIABLES WHENEVER POSSIBLE   ||
		\*====================================================================*/

		double lVel = Control.lEncoder.getRate();
		double rVel = Control.rEncoder.getRate();
		double speed = Math.abs((lVel+rVel)/2);
		Control.lcd.lines[1] = "Speed: "+Robot.strSpeed(speed);

		//</editor-fold>
	}
}
