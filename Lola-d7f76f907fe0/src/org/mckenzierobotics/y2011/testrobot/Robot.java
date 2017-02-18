package org.mckenzierobotics.y2011.testrobot;

import org.mckenzierobotics.y2011.testrobot.hardware.*;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.CANJaguar.NeutralMode;
//import org.mckenzierobotics.CAN.CANJaguar;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDOutput;
import com.sun.squawk.util.MathUtils;

/**
 *
 * @author LukeShu
 */
public class Robot {
	private static final double shift_up_speed = 4;// feet per sec
	private static final double shift_down_speed = 3;// feet per sec
	private static final double wheelCirc = .51429*Math.PI;// in feet
	
	/*<editor-fold desc="Arm">*************************************************\
	 *                               Arm                                      *
	\**************************************************************************/
	private Arm arm;
	private CANJaguar[]           a_armShoulder = new CANJaguar[2];
	private PIDOutputSplitter     m_armShoulder;
	private MagneticEncoder       e_armShoulder;
	private static final double[] s_armShoulder = {1.0, 1.0};

	private Solenoid elbowSide;
	private Solenoid elbowHeight;
	private Elbow elbow;

	private Solenoid backFinger;
	private Solenoid frontFinger;

	private Solenoid minibotArm;
	private Solenoid minibotRelease;
	//</editor-fold>

	/*<editor-fold desc="Drive">***********************************************\
	 *                              Drive                                     *
	\**************************************************************************/
	private CANJaguar[]           a_lDrive = new CANJaguar[2];
	private PIDOutput             m_lDrive;
	private DigitalInput[]       ae_lDrive = new DigitalInput[2];
	private Encoder               e_lDrive;
	private static final double[] s_lDrive = {-1.0, -1.0};

	private CANJaguar[]           a_rDrive = new CANJaguar[2];
	private PIDOutput             m_rDrive;
	private DigitalInput[]       ae_rDrive = new DigitalInput[2];
	private Encoder               e_rDrive;
	private static final double[] s_rDrive = {1.0, 1.0};

	private DigitalInput dummy0, dummy1, dummy2, dummy3;
	private Encoder e_lDummy, e_rDummy;

	private Solenoid shifter;

	private RollingAvg lSpeed_20ms;
	private RollingAvg rSpeed_20ms;
	private RollingAvg lSpeed_50ms;
	private RollingAvg rSpeed_50ms;
	double speed = 0;
	//</editor-fold>

	private Compressor comp;

	public Robot() {
		init();
	}

	public void init() {
		/*<editor-fold desc="CAN Resources">***********************************\
		 *                           CAN Resources                            *
		\**********************************************************************/
		// Constructor(CAN_address)

		/* Jaguar Layout:
		 *
		 *  +-+ +-+  |  +-+ +-+
		 *  |3| |4|  |  |7| |8|
		 *  +-+ +-+  |  +-+ +-+
		 *  +-+ +-+  |  +-+
		 *  |2| |5|  |  |6|
		 *  +-+ +-+  |  +-+
		 */

		try{ a_rDrive[0] = new CANJaguar(2); }catch(Exception e){Debug.err(e);}
		try{ a_rDrive[1] = new CANJaguar(3); }catch(Exception e){Debug.err(e);}
		try{ a_lDrive[0] = new CANJaguar(4); }catch(Exception e){Debug.err(e);}
		try{ a_lDrive[1] = new CANJaguar(5); }catch(Exception e){Debug.err(e);}
		try{ a_armShoulder[0] = new CANJaguar(6); }catch(Exception e){Debug.err(e);}
		try{ a_armShoulder[1] = new CANJaguar(7); }catch(Exception e){Debug.err(e);}

		try{ a_lDrive[0].configNeutralMode(NeutralMode.kCoast); }catch(Exception e){Debug.err(e);}
		try{ a_lDrive[1].configNeutralMode(NeutralMode.kCoast); }catch(Exception e){Debug.err(e);}
		try{ a_rDrive[0].configNeutralMode(NeutralMode.kCoast); }catch(Exception e){Debug.err(e);}
		try{ a_rDrive[1].configNeutralMode(NeutralMode.kCoast); }catch(Exception e){Debug.err(e);}
		try{ a_armShoulder[0].configNeutralMode(NeutralMode.kBrake); }catch(Exception e){Debug.err(e);}
		try{ a_armShoulder[1].configNeutralMode(NeutralMode.kBrake); }catch(Exception e){Debug.err(e);}

		m_armShoulder = new PIDOutputSplitter(a_armShoulder, s_armShoulder);
		m_lDrive = new PIDOutputSplitter(a_lDrive, s_lDrive);
		m_rDrive = new PIDOutputSplitter(a_rDrive, s_rDrive);

		//</editor-fold>

		/*<editor-fold desc="Digital Resources">*******************************\
		 *                           Digital Resources                        *
		\**********************************************************************/
		// Constructor(channel1 [, channel2 [, channel3 [...]]])
		comp         = new Compressor(8,1);// Switch (DIO), Relay (relay)
		dummy0       = new DigitalInput(2);
		dummy1       = new DigitalInput(3);
		dummy2       = new DigitalInput(4);
		dummy3       = new DigitalInput(5);
		ae_lDrive[0] = new DigitalInput(6);
		ae_lDrive[1] = new DigitalInput(7);
		ae_rDrive[0] = new DigitalInput(10);
		ae_rDrive[1] = new DigitalInput(11);

		e_lDummy = new Encoder(dummy0, dummy1);
		e_lDrive = new Encoder(ae_lDrive[0],ae_lDrive[1]);
		e_rDummy = new Encoder(dummy2, dummy3);
		e_rDrive = new Encoder(ae_rDrive[0],ae_rDrive[1]);

		e_lDrive.start();
		e_rDrive.start();
		e_lDrive.setDistancePerPulse(wheelCirc/250.0);
		e_rDrive.setDistancePerPulse(wheelCirc/250.0);

		Control.lEncoder = e_lDrive;
		Control.rEncoder = e_rDrive;

		lSpeed_20ms = new RollingAvg(10, new PIDEncoderSpeed(e_lDrive));
		rSpeed_20ms = new RollingAvg(10, new PIDEncoderSpeed(e_rDrive));
		lSpeed_50ms = new RollingAvg(10, new PIDEncoderSpeed(e_lDrive));
		rSpeed_50ms = new RollingAvg(10, new PIDEncoderSpeed(e_rDrive));


		Control.lSpeed = new PIDController(.6,.05,1, lSpeed_50ms, m_lDrive);
		Control.rSpeed = new PIDController(.6,.05,1, rSpeed_50ms, m_rDrive);

		Control.lSpeed.disable();
		Control.rSpeed.disable();
		Control.lSpeed.setInputRange(-3.5, 3.5);
		Control.rSpeed.setInputRange(-3.5, 3.5);
		Control.lSpeed.setOutputRange(-1,1);
		Control.rSpeed.setOutputRange(-1,1);

		//</editor-fold>

		/*<editor-fold desc="High Voltage Digital Out Resources">**************\
		 *                  High Voltage Digital Out Resources                *
		\**********************************************************************/
		frontFinger    = new Solenoid(1);
		backFinger     = new Solenoid(2);
		shifter        = new Solenoid(3);
		elbowSide      = new Solenoid(4);
		elbowHeight    = new Solenoid(5);
		minibotArm     = new Solenoid(6);
		minibotRelease = new Solenoid(7);

		//</editor-fold>

		/*<editor-fold desc="Analog Resources">********************************\
		 *                          Analog Resources                          *
		\**********************************************************************/
		// Constructor(slot, channel)
		e_armShoulder = new MagneticEncoder(1, 1, /*2.5*/ 3.35);

		//</editor-fold>

		/*<editor-fold desc="Misc">********************************************\
		 *                                Misc.                               *
		\**********************************************************************/
		elbow = new ElbowPneumaticDual(elbowSide, elbowHeight);
		arm = new Arm(e_armShoulder, m_armShoulder, elbow);
		//</editor-fold>
	}

	public void periodic() {
		if (Control.calibrate) {
			// For the first time ever, I am comfortable with this number
			e_armShoulder.calibrateDegrees(158*(Control.armFront?1:-1));
		}
		if (Control.calibrate0) {
			e_armShoulder.calibrateDegrees(0);
		}
		arm.periodic(
			Control.armMode, Control.armMiddle, (Control.armFront?1:-1),// PID Control
			Control.armShoulder,  Control.armElbow // Manual Control
		);
		Control.armInPosition = arm.isInPosition();

		backFinger.set(Control.backFingerOpen);
		frontFinger.set(Control.frontFingerOpen);

		//<editor-fold desc="Compressor">
		if (comp.getPressureSwitchValue()) {
			comp.stop();
		} else {
			comp.start();
		}
		//</editor-fold>

		minibotArm.set(Control.minibotArmOut);
		minibotRelease.set(Control.minibotRelease);

		//<editor-fold desc="Shifter">
		double lSpeed = lSpeed_20ms.pidGet();
		double rSpeed = rSpeed_20ms.pidGet();
		speed = Math.max(
				Math.abs(lSpeed),
				Math.abs(rSpeed)
			);

		boolean shift = false;
		switch (Control.shift) {
			case 0: // auto
				if (shifter.get()) {
					shift = speed>shift_down_speed;
				} else {
					shift = speed>shift_up_speed;
				}
				break;
			case 1: // low
				shift = false;
				break;
			case 2: // high
				shift = true;
				break;
		}
		shifter.set(shift);
		//</editor-fold>

		//<editor-fold desc="Drive">
		if (!Control.lSpeed.isEnable()) m_lDrive.pidWrite(Control.lPower);
		if (!Control.rSpeed.isEnable()) m_rDrive.pidWrite(Control.rPower);
		//</editor-fold>

		/*<editor-fold desc="DriverStationLCD">********************************\
		 *                         DriverStationLCD                           *
		\**********************************************************************/

		/*====================================================================*\
		||         NOTE: USE `Control.whatever' VARIABLES WHENEVER POSSIBLE   ||
		\*====================================================================*/
		//Control.lcd.lines[0] = // reserved for Debug.err(e)
		//Control.lcd.lines[1] = // reserved for the controller
		Control.lcd.lines[2] = strCompressor();
		Control.lcd.lines[3] = strShifter();
		Control.lcd.lines[4] = strShoulder();
		Control.lcd.lines[5] = strElbow();

		//</editor-fold>
	}

	public String strCompressor() {
		return "compressor: "+(comp.enabled()?"on":"off");
	}

	public String strShifter() {
		String shiftStr = "";
		switch (Control.shift) {
			case 0: shiftStr="auto"; break;
			case 1: shiftStr="low"; break;
			case 2: shiftStr="high"; break;
			default: shiftStr="WTF!?"; break;
		}
		return "shifter: "+shiftStr;

	}

	public static String strSpeed(double speed) {
		return (MathUtils.round(speed*100)/100.0)+" ft/s";

	}

	public String strShoulder() {
		return "s: "
				+ MathUtils.round(e_armShoulder.pidGet())
				+" : "
				+e_armShoulder.getRawVoltage();
	}

	public String strElbow() {
		String elbowStr = "";
		switch (elbow.get()) {
			case 0: elbowStr = "_. "; break;
			case 1: elbowStr = "\\. "; break;
			case 2: elbowStr = " ./"; break;
			case 3: elbowStr = " ._"; break;
			default: break;
		}
		return "e: " + elbow.get()+": c]"+elbowStr+"[b";
	}
}
