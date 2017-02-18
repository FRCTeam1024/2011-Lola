package org.mckenzierobotics.CAN;

import java.util.Vector;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.CANJaguar.ControlMode;
import edu.wpi.first.wpilibj.CANJaguar.SpeedReference;
import edu.wpi.first.wpilibj.CANJaguar.PositionReference;
import edu.wpi.first.wpilibj.CANJaguar.NeutralMode;
import edu.wpi.first.wpilibj.can.JaguarCANProtocol;

/**
 * A Plug-`n`-play version of the CANJaguar driver.  It is 100%
 * source-compatible with the CANJaguar driver in the same version of WPIlibj.
 * However, do to some of Java's brain-dead-nes, PnPCANJaguar cannot
 * extend/implement CANJaguar.
 *
 * Even though you won't be importing `edu.wpi.first.wpilibj.CANJaguar', you
 * may want to import some of its sub-classes (such as NeutralMode), which are
 * NOT reproduced in PnPCANJaguar.
 *
 * @author LukeShumaker
 * @author Some folks from WPI
 * @version 2011.7
 */
public class PnPCANJaguar implements MotorSafety, PIDOutput, SpeedController {

	public static interface Action {
		public void doit(CANJaguar jag) throws Exception;
	}

	protected CANJaguar actualJag = null;

	private boolean saidCantInit = false;
	private final int i_deviceNumber;
	private final ControlMode i_controlMode;
	private final Vector actionStack = new Vector();
	public void doStack() {
		if (actualJag == null) {
			try {
				actualJag = new CANJaguar(i_deviceNumber, i_controlMode);
			} catch (Exception e) {
				if (!saidCantInit) {
					System.out.println("PnPCANJaguar["+i_deviceNumber+"]: "+
							"could not init: "+ e.getMessage());
					saidCantInit = true;
				}
			}
		}
		if (actualJag != null) {
			while (!actionStack.isEmpty()) {
				Action action = (Action)actionStack.firstElement();
				try {
					action.doit(actualJag);
				} catch (Exception e) {
					System.out.println("PnPCANJaguar["+i_deviceNumber+"]: "+
							e.getMessage());
					return;
				}
				actionStack.removeElementAt(0);
			}
		}
	}
	public void stackPush(Action action) {
		actionStack.addElement(action);
	}

	/**
	 * Constructor
	 * Default to percent Vbus control mode.
	 * @param deviceNumber The address of the Jaguar on the CAN bus.
	 */

	public PnPCANJaguar(int deviceNumber) {
		this(deviceNumber, ControlMode.kPercentVbus);
	}

	/**
	 * Constructor
	 * @param deviceNumber The address of the Jaguar on the CAN bus.
	 * @param controlMode The control mode that the Jaguar will run in.
	 */
	public PnPCANJaguar(int deviceNumber, ControlMode controlMode) {
		i_deviceNumber = deviceNumber;
		i_controlMode = controlMode;
		doStack();
	}

	/**
	 * Set the output set-point value.
	 *
	 * The scale and the units depend on the mode the Jaguar is in.
	 * In PercentVbus Mode, the outputValue is from -1.0 to 1.0 (same as PWM Jaguar).
	 * In Voltage Mode, the outputValue is in Volts.
	 * In Current Mode, the outputValue is in Amps.
	 * In Speed Mode, the outputValue is in Rotations/Minute.
	 * In Position Mode, the outputValue is in Rotations.
	 *
	 * @param outputValue The set-point to sent to the motor controller.
	 */
	public void setX(double outputValue) {
		setX(outputValue, (byte) 0);
	}
	
	/**
	 * Set the output set-point value.
	 *
	 * Needed by the SpeedControl interface (swallows CANTimeoutExceptions).
	 *
	 * @deprecated Use setX instead.
	 * @param outputValue The set-point to sent to the motor controller.
	 */
	public void set(double outputValue) {
		set(outputValue, (byte) 0);
	}

	/**
	 * Set the output set-point value.
	 *
	 * The scale and the units depend on the mode the Jaguar is in.
	 * In PercentVbus Mode, the outputValue is from -1.0 to 1.0 (same as PWM Jaguar).
	 * In Voltage Mode, the outputValue is in Volts.
	 * In Current Mode, the outputValue is in Amps.
	 * In Speed Mode, the outputValue is in Rotations/Minute.
	 * In Position Mode, the outputValue is in Rotations.
	 *
	 * @param outputValue The set-point to sent to the motor controller.
	 * @param syncGroup The update group to add this set() to, pending updateSyncGroup().  If 0, update immediately.
	 */
	public void setX(double outputValue, byte syncGroup) {
		stackPush(new Action_setX(outputValue, syncGroup));
		doStack();
	}
	public static class Action_setX implements Action {
		private final double outputValue;
		private final byte syncGroup;
		public Action_setX(double outputValue, byte syncGroup) {
			this.outputValue = outputValue;
			this.syncGroup = syncGroup;
		}
		public void doit(CANJaguar jag) throws Exception {
			jag.setX(outputValue, syncGroup);
		}
	}

	/**
	 * Set the output set-point value.
	 *
	 * Needed by the SpeedControl interface (swallows CANTimeoutExceptions).
	 *
	 * @deprecated Use setX instead.
	 * @param outputValue The set-point to sent to the motor controller.
	 * @param syncGroup The update group to add this set() to, pending updateSyncGroup().  If 0, update immediately.
	 */
	public void set(double outputValue, byte syncGroup) {
		setX(outputValue, syncGroup);
	}

	/**
	 * Get the recently set outputValue setpoint.
	 *
	 * The scale and the units depend on the mode the Jaguar is in.
	 * In PercentVbus Mode, the outputValue is from -1.0 to 1.0 (same as PWM Jaguar).
	 * In Voltage Mode, the outputValue is in Volts.
	 * In Current Mode, the outputValue is in Amps.
	 * In Speed Mode, the outputValue is in Rotations/Minute.
	 * In Position Mode, the outputValue is in Rotations.
	 *
	 * @return The most recently set outputValue setpoint.
	 */
	public double getX() {
		stackPush(new Action_getX());
		doStack();
		return Action_getX.last;
	}
	public static class Action_getX implements Action {
		public static double last = 0.0;
		public Action_getX() {}
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getX();
		}
	}

	/**
	 * Get the recently set outputValue setpoint.
	 *
	 * Needed by the SpeedControl interface (swallows CANTimeoutExceptions).
	 *
	 * @deprecated Use getX instead.
	 * @return The most recently set outputValue setpoint.
	 */
	public double get() {
		return getX();
	}

	/**
	 * Common interface for disabling a motor.
	 *
	 * Needed by the SpeedControl interface (swallows CANTimeoutExceptions).
	 *
	 * @deprecated Use disableControl instead.
	 */
	public void disable() {
		disableControl();
	}

	/**
	 * Write out the PID value as seen in the PIDOutput base object.
	 *
	 * @deprecated Use setX instead.
	 * @param output Write out the percentage voltage value as was computed by the PIDController
	 */
	public void pidWrite(double output) {
		if (Action_getControlMode.last == ControlMode.kPercentVbus) {
			set(output);
		} else {
			// TODO: Error... only percent vbus mode supported for PID API
		}
	}

	/**
	 * Set the reference source device for speed controller mode.
	 *
	 * Choose encoder as the source of speed feedback when in speed control mode.
	 *
	 * @param reference Specify a SpeedReference.
	 */
	public void setSpeedReference(SpeedReference reference) {
		stackPush(new Action_setSpeedReference(reference));
		doStack();
	}
	public static class Action_setSpeedReference implements Action {
		private final SpeedReference reference;
		public Action_setSpeedReference(SpeedReference reference) {
			this.reference = reference;
		}
		public void doit(CANJaguar jag) throws Exception {
			jag.setSpeedReference(reference);
			Action_getSpeedReference.last = reference;
		}
	}

	/**
	 * Get the reference source device for speed controller mode.
	 *
	 * @return A SpeedReference indicating the currently selected reference device for speed controller mode.
	 */
	public SpeedReference getSpeedReference() {
		stackPush(new Action_getSpeedReference());
		doStack();
		return Action_getSpeedReference.last;
	}
	public static class Action_getSpeedReference implements Action {
		public static SpeedReference last = SpeedReference.kNone;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getSpeedReference();
		}
	}

	/**
	 * Set the reference source device for position controller mode.
	 *
	 * Choose between using and encoder and using a potentiometer
	 * as the source of position feedback when in position control mode.
	 *
	 * @param reference Specify a PositionReference.
	 */
	public void setPositionReference(PositionReference reference) {
		stackPush(new Action_setPositionReference(reference));
		doStack();
	}
	public static class Action_setPositionReference implements Action {
		private final PositionReference reference;
		public Action_setPositionReference(PositionReference reference) {
			this.reference = reference;
		}
		public void doit(CANJaguar jag) throws Exception {
			jag.setPositionReference(reference);
			Action_getPositionReference.last = reference;
		}
	}

	/**
	 * Get the reference source device for position controller mode.
	 *
	 * @return A PositionReference indicating the currently selected reference device for position controller mode.
	 */
	public PositionReference getPositionReference() {
		stackPush(new Action_getPositionReference());
		doStack();
		return Action_getPositionReference.last;
	}
	public static class Action_getPositionReference implements Action {
		public static PositionReference last = PositionReference.kNone;;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getPositionReference();
		}
	}

	/**
	 * Set the P, I, and D constants for the closed loop modes.
	 *
	 * @param p The proportional gain of the Jaguar's PID controller.
	 * @param i The integral gain of the Jaguar's PID controller.
	 * @param d The differential gain of the Jaguar's PID controller.
	 */
	public void setPID(double p, double i, double d) {
		stackPush(new Action_setPID(p, i, d));
	}
	public static class Action_setPID implements Action {
		private final double p, i, d;
		public Action_setPID(double p, double i, double d) {
			this.p = p;
			this.i = i;
			this.d = d;
		}
		public void doit(CANJaguar jag) throws Exception {
			jag.setPID(p, i, d);
			Action_getP.last = p;
			Action_getI.last = i;
			Action_getD.last = d;
		}
	}

	/**
	 * Get the Proportional gain of the controller.
	 *
	 * @return The proportional gain.
	 */
	public double getP() {
		stackPush(new Action_getP());
		doStack();
		return Action_getP.last;
	}
	public static class Action_getP implements Action {
		public static double last = 0;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getP();
		}
	}

	/**
	 * Get the Intregral gain of the controller.
	 *
	 * @return The integral gain.
	 */
	public double getI() {
		stackPush(new Action_getI());
		doStack();
		return Action_getI.last;
	}
	public static class Action_getI implements Action {
		public static double last = 0;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getI();
		}
	}

	/**
	 * Get the Differential gain of the controller.
	 *
	 * @return The differential gain.
	 */
	public double getD() {
		stackPush(new Action_getD());
		doStack();
		return Action_getD.last;
	}
	public static class Action_getD implements Action {
		public static double last = 0;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getD();
		}
	}

	/**
	 * Enable the closed loop controller.
	 *
	 * Start actually controlling the output based on the feedback.
	 */
	public void enableControl() {
		enableControl(0.0);
	}

	/**
	 * Enable the closed loop controller.
	 *
	 * Start actually controlling the output based on the feedback.
	 * If starting a position controller with an encoder reference,
	 * use the encoderInitialPosition parameter to initialize the
	 * encoder state.
	 * @param encoderInitialPosition Encoder position to set if position with encoder reference.  Ignored otherwise.
	 */
	public void enableControl(double encoderInitialPosition) {
		stackPush(new Action_enableControl(encoderInitialPosition));
		doStack();
	}
	public static class Action_enableControl implements Action {
		double encoderInitialPosition;
		public Action_enableControl(double encoderInitialPosition) {
			this.encoderInitialPosition = encoderInitialPosition;
		}
		public void doit(CANJaguar jag) throws Exception {
			jag.enableControl(encoderInitialPosition);
		}
	}

	/**
	 * Disable the closed loop controller.
	 *
	 * Stop driving the output based on the feedback.
	 */
	public void disableControl() {
		stackPush(new Action_disableControl());
		doStack();
	}
	public static class Action_disableControl implements Action {
		public void doit(CANJaguar jag) throws Exception {
			jag.disableControl();
		}
	}

	/**
	 * Change the control mode of this Jaguar object.
	 *
	 * After changing modes, configure any PID constants or other settings needed
	 * and then enableControl() to actually change the mode on the Jaguar.
	 *
	 * @param controlMode The new mode.
	 */
	public void changeControlMode(ControlMode controlMode) {
		stackPush(new Action_changeControlMode(controlMode));
		doStack();
	}
	public static class Action_changeControlMode implements Action {
		private final ControlMode controlMode;
		public Action_changeControlMode(ControlMode controlMode) {
			this.controlMode = controlMode;
		}
		public void doit(CANJaguar jag) throws Exception {
			jag.changeControlMode(controlMode);
			Action_getControlMode.last = controlMode;
		}
	}

	/**
	 * Get the active control mode from the Jaguar.
	 *
	 * Ask the Jag what mode it is in.
	 *
	 * @return ControlMode that the Jag is in.
	 */
	public ControlMode getControlMode() {
		stackPush(new Action_getControlMode());
		doStack();
		return Action_getControlMode.last;
	}
	public static class Action_getControlMode implements Action {
		public static ControlMode last = ControlMode.kPercentVbus;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getControlMode();
		}
	}

	/**
	 * Get the voltage at the battery input terminals of the Jaguar.
	 *
	 * @return The bus voltage in Volts.
	 */
	public double getBusVoltage() {
		stackPush(new Action_getBusVoltage());
		doStack();
		return Action_getBusVoltage.last;
	}
	public static class Action_getBusVoltage implements Action {
		public static double last = 0.0;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getBusVoltage();
		}
	}

	/**
	 * Get the voltage being output from the motor terminals of the Jaguar.
	 *
	 * @return The output voltage in Volts.
	 */
	public double getOutputVoltage() {
		stackPush(new Action_getOutputVoltage());
		doStack();
		return Action_getOutputVoltage.last;
	}
	public static class Action_getOutputVoltage implements Action {
		public static double last = 0.0;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getOutputVoltage();
		}
	}

	/**
	 * Get the current through the motor terminals of the Jaguar.
	 *
	 * @return The output current in Amps.
	 */
	public double getOutputCurrent() {
		stackPush(new Action_getOutputCurrent());
		doStack();
		return Action_getOutputCurrent.last;
	}
	public static class Action_getOutputCurrent implements Action {
		public static double last = 0.0;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getOutputCurrent();
		}
	}

	/**
	 * Get the internal temperature of the Jaguar.
	 *
	 * @return The temperature of the Jaguar in degrees Celsius.
	 */
	public double getTemperature() {
		stackPush(new Action_getTemperature());
		doStack();
		return Action_getTemperature.last;
	}
	public static class Action_getTemperature implements Action {
		public static double last = 0.0;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getTemperature();
		}
	}

	/**
	 * Get the position of the encoder or potentiometer.
	 *
	 * @return The position of the motor based on the configured feedback.
	 */
	public double getPosition() {
		stackPush(new Action_getPosition());
		doStack();
		return Action_getPosition.last;
	}
	public static class Action_getPosition implements Action {
		public static double last = 0.0;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getPosition();
		}
	}

	/**
	 * Get the speed of the encoder.
	 *
	 * @return The speed of the motor in RPM based on the configured feedback.
	 */
	public double getSpeed() {
		stackPush(new Action_getSpeed());
		doStack();
		return Action_getSpeed.last;
	}
	public static class Action_getSpeed implements Action {
		public static double last = 0.0;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getSpeed();
		}
	}


	/**
	 * Get the status of the forward limit switch.
	 *
	 * @return The motor is allowed to turn in the forward direction when true.
	 */
	public boolean getForwardLimitOK() {
		stackPush(new Action_getForwardLimitOK());
		doStack();
		return Action_getForwardLimitOK.last;
	}
	public static class Action_getForwardLimitOK implements Action {
		public static boolean last = false;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getForwardLimitOK();
		}
	}

	/**
	 * Get the status of the reverse limit switch.
	 *
	 * @return The motor is allowed to turn in the reverse direction when true.
	 */
	public boolean getReverseLimitOK() {
		stackPush(new Action_getReverseLimitOK());
		doStack();
		return Action_getReverseLimitOK.last;
	}
	public static class Action_getReverseLimitOK implements Action {
		public static boolean last = false;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getReverseLimitOK();
		}
	}

	/**
	 * Get the status of any faults the Jaguar has detected.
	 *
	 * @return A bit-mask of faults defined by the "Faults" enum class.
	 */
	public short getFaults() {
		stackPush(new Action_getFaults());
		doStack();
		return Action_getFaults.last;
	}
	public static class Action_getFaults implements Action {
		public static short last = 0;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getFaults();
		}
	}

	/**
	 * Check if the Jaguar's power has been cycled since this was last called.
	 *
	 * This should return true the first time called after a Jaguar power up,
	 * and false after that.
	 *
	 * @return The Jaguar was power cycled since the last call to this function.
	 */
	public boolean getPowerCycled() {
		stackPush(new Action_getPowerCycled());
		doStack();
		return Action_getPowerCycled.last;
	}
	public static class Action_getPowerCycled implements Action {
		public static boolean last = false;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getPowerCycled();
		}
	}

	/**
	 * Set the maximum voltage change rate.
	 *
	 * When in percent voltage output mode, the rate at which the voltage changes can
	 * be limited to reduce current spikes.  Set this to 0.0 to disable rate limiting.
	 *
	 * @param rampRate The maximum rate of voltage change in Percent Voltage mode in V/s.
	 */
	public void setVoltageRampRate(double rampRate) {
		stackPush(new Action_setVoltageRampRate(rampRate));
		doStack();
	}
	public static class Action_setVoltageRampRate implements Action {
		private final double rampRate;
		public Action_setVoltageRampRate(double rampRate) {
			this.rampRate = rampRate;
		}
		public void doit(CANJaguar jag) throws Exception {
			jag.setVoltageRampRate(rampRate);
		}
	}

	/**
	 * Get the version of the firmware running on the Jaguar.
	 *
	 * @return The firmware version.  0 if the device did not respond.
	 */
	public int getFirmwareVersion() {
		stackPush(new Action_getFirmwareVersion());
		doStack();
		return Action_getFirmwareVersion.last;
	}
	public static class Action_getFirmwareVersion implements Action {
		public static int last = 0;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getFirmwareVersion();
		}
	}

	/**
	 * Get the version of the Jaguar hardware.
	 *
	 * @return The hardware version. 1: Jaguar,  2: Black Jaguar
	 */
	public byte getHardwareVersion() {
		stackPush(new Action_getHardwareVersion());
		doStack();
		return Action_getHardwareVersion.last;
	}
	public static class Action_getHardwareVersion implements Action {
		// Assume Gray Jag if there is no response
		public static byte last = JaguarCANProtocol.LM_HWVER_JAG_1_0;
		public void doit(CANJaguar jag) throws Exception {
			last = jag.getHardwareVersion();
		}
	}

	/**
	 * Configure what the controller does to the H-Bridge when neutral (not driving the output).
	 *
	 * This allows you to override the jumper configuration for brake or coast.
	 *
	 * @param mode Select to use the jumper setting or to override it to coast or brake.
	 */
	public void configNeutralMode(NeutralMode mode) {
		stackPush(new Action_configNeutralMode(mode));
		doStack();
	}
	public static class Action_configNeutralMode implements Action {
		private final NeutralMode mode;
		public Action_configNeutralMode(NeutralMode mode) {
			this.mode = mode;
		}
		public void doit(CANJaguar jag) throws Exception {
			jag.configNeutralMode(mode);
		}
	}

	/**
	 * Configure how many codes per revolution are generated by your encoder.
	 *
	 * @param codesPerRev The number of counts per revolution in 1X mode.
	 */
	public void configEncoderCodesPerRev(int codesPerRev) {
		stackPush(new Action_configEncoderCodesPerRev(codesPerRev));
		doStack();
	}
	public static class Action_configEncoderCodesPerRev implements Action {
		private final int codesPerRev;
		public Action_configEncoderCodesPerRev(int codesPerRev) {
			this.codesPerRev = codesPerRev;
		}
		public void doit(CANJaguar jag) throws Exception {
			jag.configEncoderCodesPerRev(codesPerRev);
		}
	}

	/**
	 * Configure the number of turns on the potentiometer.
	 *
	 * There is no special support for continuous turn potentiometers.
	 * Only integer numbers of turns are supported.
	 *
	 * @param turns The number of turns of the potentiometer
	 */
	public void configPotentiometerTurns(int turns) {
		stackPush(new Action_configPotentiometerTurns(turns));
		doStack();
	}
	public static class Action_configPotentiometerTurns implements Action {
		private final int turns;
		public Action_configPotentiometerTurns(int turns) {
			this.turns = turns;
		}
		public void doit(CANJaguar jag) throws Exception {
			jag.configPotentiometerTurns(turns);
		}
	}

	/**
	 * Configure Soft Position Limits when in Position Controller mode.
	 *
	 * When controlling position, you can add additional limits on top of the limit switch inputs
	 * that are based on the position feedback.  If the position limit is reached or the
	 * switch is opened, that direction will be disabled.
	 *
	 * @param forwardLimitPosition The position that if exceeded will disable the forward direction.
	 * @param reverseLimitPosition The position that if exceeded will disable the reverse direction.
	 */
	public void configSoftPositionLimits(double forwardLimitPosition, double reverseLimitPosition) {
		stackPush(new Action_configSoftPositionLimits(forwardLimitPosition, reverseLimitPosition));
		doStack();
	}
	public static class Action_configSoftPositionLimits implements Action {
		private final double forwardLimitPosition, reverseLimitPosition;
		public Action_configSoftPositionLimits(double forwardLimitPosition, double reverseLimitPosition) {
			this.forwardLimitPosition = forwardLimitPosition;
			this.reverseLimitPosition = reverseLimitPosition;
		}
		public void doit(CANJaguar jag) throws Exception {
			jag.configSoftPositionLimits(forwardLimitPosition, reverseLimitPosition);
		}
	}

	/**
	 * Disable Soft Position Limits if previously enabled.
	 *
	 * Soft Position Limits are disabled by default.
	 */
	public void disableSoftPositionLimits() {
		stackPush(new Action_disableSoftPositionLimits());
		doStack();
	}
	public static class Action_disableSoftPositionLimits implements Action {
		public void doit(CANJaguar jag) throws Exception {
			jag.disableSoftPositionLimits();
		}
	}

	/**
	 * Configure the maximum voltage that the Jaguar will ever output.
	 *
	 * This can be used to limit the maximum output voltage in all modes so that
	 * motors which cannot withstand full bus voltage can be used safely.
	 *
	 * @param voltage The maximum voltage output by the Jaguar.
	 */
	public void configMaxOutputVoltage(double voltage) {
		stackPush(new Action_configMaxOutputVoltage(voltage));
		doStack();
	}
	public static class Action_configMaxOutputVoltage implements Action {
		private final double voltage;
		public Action_configMaxOutputVoltage(double voltage) {
			this.voltage = voltage;
		}
		public void doit(CANJaguar jag) throws Exception {
			jag.configMaxOutputVoltage(voltage);
		}
	}

	/**
	 * Configure how long the Jaguar waits in the case of a fault before resuming operation.
	 *
	 * Faults include over temerature, over current, and bus under voltage.
	 * The default is 3.0 seconds, but can be reduced to as low as 0.5 seconds.
	 *
	 * @param faultTime The time to wait before resuming operation, in seconds.
	 */
	public void configFaultTime(double faultTime) {
		stackPush(new Action_configFaultTime(faultTime));
		doStack();
	}
	public static class Action_configFaultTime implements Action {
		private final double faultTime;
		public Action_configFaultTime(double faultTime) {
			this.faultTime = faultTime;
		}
		public void doit(CANJaguar jag) throws Exception {
			jag.configFaultTime(faultTime);
		}
	}

	public static void updateSyncGroup(byte syncGroup) {
        try {
			CANJaguar.updateSyncGroup(syncGroup);
		} catch (Exception e) {
			System.out.println("PnPCANJaguar: could not updateSyncGroup: "+
					e.getMessage());
		}
    }

	public void setExpiration(double timeout) {
		stackPush(new Action_setExpiration(timeout));
		doStack();
	}
	public static class Action_setExpiration implements Action {
		private final double timeout;
		public Action_setExpiration(double timeout) {
			this.timeout = timeout;
		}
		public void doit(CANJaguar jag) throws Exception {
			jag.setExpiration(timeout);
			Action_getExpiration.last = timeout;
		}
	}

	public double getExpiration() {
		stackPush(new Action_getExpiration());
		doStack();
		return Action_getExpiration.last;
	}
	public static class Action_getExpiration implements Action {
		public static double last = 0;// XXX: correct default?
		public void doit(CANJaguar jag) {
			last = jag.getExpiration();
		}
	}

	public boolean isAlive() {
		stackPush(new Action_isAlive());
		doStack();
		return Action_isAlive.last;
	}
	public static class Action_isAlive implements Action {
		public static boolean last = false;// XXX: correct defualt?
		public void doit(CANJaguar jag) {
			last = jag.isAlive();
		}
	}

	public boolean isSafetyEnabled() {
		stackPush(new Action_isSafetyEnabled());
		doStack();
		return Action_isSafetyEnabled.last;
	}
	public static class Action_isSafetyEnabled implements Action {
		public static boolean last = false;// XXX: correct defualt?
		public void doit(CANJaguar jag) {
			last = jag.isSafetyEnabled();
		}
	}

	public void setSafetyEnabled(boolean enabled) {
		stackPush(new Action_setSafetyEnabled(enabled));
		doStack();
	}
	public static class Action_setSafetyEnabled implements Action {
		private final boolean enabled;
		public Action_setSafetyEnabled(boolean enabled) {
			this.enabled = enabled;
		}
		public void doit(CANJaguar jag) throws Exception {
			jag.setSafetyEnabled(enabled);
			Action_isSafetyEnabled.last = enabled;
		}
	}

    /**
     * Common interface for stopping a motor.
     *
     * @deprecated Use disableControl instead.
     */
    public void stopMotor() {
        disableControl();
    }

}
