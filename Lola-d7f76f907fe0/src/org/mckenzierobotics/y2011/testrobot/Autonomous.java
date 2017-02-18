package org.mckenzierobotics.y2011.testrobot;

import org.mckenzierobotics.y2011.testrobot.hardware.Arm;

/**
 *
 * @author LukeShu
 */
public class Autonomous implements Controller {
	/** how many tubes to go for */
	private static final int tubes = 1;
	/** Wait for the arm to get to the (t)op? */
	private static final boolean tWait = false;
	/** Wait for the arm to get to the (f)loor? */
	private static final boolean fWait = false;
	/** Use speed control (experimental)? */
	private static final boolean speedControl = false;

	////////////////////////////////////////////////////////////////////////////
	
	private final RALFF ralff = new RALFF();
	private int stage = 0;

	public void init() {
		stage = 0;
	}

	public void periodic() {
		
		Control.lcd.lines[1] = "stage: "+stage;
		if (speedControl) {
			periodicSpeedControl();
		} else {
			periodicPowerControl();
		}
	}

	public void periodicSpeedControl() {
		switch (stage) {
			case  0: if (ralff.init()                            )stage++;break;
			case  1: if (ralff.shiftLow()                        )stage++;break;
			case  2: if (ralff.setHand(false, false)             )stage++;break;
			case  3: if (ralff.setForward(false)                 )stage++;break;
			case  4: if (ralff.calibrateArm()                    )stage++;break;
			// Score ////////////////////////////////////////////)stage++;break;
			case  5: if (tubes >= 1                              )stage++;break;
			case  6: if (ralff.setForward(false)                 )stage++;break;
			case  7: Control.armMiddle = false                   ;stage++;break;
			case  8: if (ralff.moveArm(Arm.Mode.kTop,false)      )stage++;break;
			case  9: if (ralff.drivePID(-11.25,-11, 2.5)         )stage++;break;
			case 10: if (ralff.moveArm(Arm.Mode.kTop,tWait)      )stage++;break;
			case 11: if (ralff.drivePID(-2.7,-2.7, 1)            )stage++;break;
			case 12: if (ralff.moveArm(null,false)               )stage++;break;
			case 13: if (ralff.sleep(1)                          )stage++;break;
			case 14: if (ralff.setHand(false, true)              )stage++;break;
			case 15: if (ralff.sleep(.5)                         )stage++;break;
			case 16: if (ralff.driveDistancePower(2,2, 1)        )stage++;break;
			case 17:                                              stage++;break;
			case 18: if (ralff.setForward(true)                  )stage++;break;
			case 19: if (ralff.moveArm(Arm.Mode.kTransport,false))stage++;break;
			// Pick up second tube //////////////////////////////)stage++;break;
			case 20: if (tubes >= 2                              )stage++;break;
			case 21: Control.armMiddle=true                      ;stage++;break;
			case 22: if (ralff.moveArm(Arm.Mode.kLoadFloor,fWait))stage++;break;
			case 23:                                              stage++;break;
			case 24: if (ralff.drivePID(4.7,6, 2.5)              )stage++;break;
			case 25: if (ralff.setHand(false, false)             )stage++;break;
			case 26: if (ralff.moveArm(Arm.Mode.kTransport,false))stage++;break;
			// Score second tube ////////////////////////////////)stage++;break;
			case 27: if (ralff.setForward(false)                 )stage++;break;
			case 28: if (ralff.moveArm(Arm.Mode.kTop,false)      )stage++;break;
			case 29: if (ralff.drivePID(0,-1.5, 1)               )stage++;break;
			case 30: if (ralff.drivePID(-4,-4, 2.5)              )stage++;break;
			case 31: if (ralff.moveArm(Arm.Mode.kTop,tWait)      )stage++;break;
			case 32: if (ralff.drivePID(-2.7,-2.7, 1)            )stage++;break;
			case 33: if (ralff.moveArm(null,false)               )stage++;break;
			case 34: if (ralff.sleep(1)                          )stage++;break;
			case 35: if (ralff.setHand(false, true)              )stage++;break;
			case 36: if (ralff.sleep(.5)                         )stage++;break;
			case 37: if (ralff.drivePID(2,2, 1)                  )stage++;break;
			case 38:                                              stage++;break;
			case 39: if (ralff.setForward(true)                  )stage++;break;
			case 40: if (ralff.moveArm(Arm.Mode.kTransport,false))stage++;break;
			// Pick up third tube ///////////////////////////////)stage++;break;
			case 41: if (tubes >= 3                              )stage++;break;
			case 42: if (ralff.moveArm(Arm.Mode.kLoadFloor,fWait))stage++;break;
			case 43:                                              stage++;break;
			case 44: if (ralff.drivePID(5,7, 1)                  )stage++;break;
			case 45: if (ralff.setHand(false, false)             )stage++;break;
			case 46: if (ralff.moveArm(Arm.Mode.kTransport,false))stage++;break;
			// Score third tube /////////////////////////////////)stage++;break;
			case 47: if (ralff.setForward(false)                 )stage++;break;
			case 48: if (ralff.moveArm(Arm.Mode.kTop,false)      )stage++;break;
			case 49: if (ralff.drivePID(0,-1.7, 1)               )stage++;break;
			case 50: if (ralff.drivePID(-4,-4.5, 2)              )stage++;break;
			case 51: if (ralff.moveArm(Arm.Mode.kTop,tWait)      )stage++;break;
			case 52: if (ralff.drivePID(-2,-2, 1)                )stage++;break;
			case 53: if (ralff.moveArm(null,false)               )stage++;break;
			case 54: if (ralff.sleep(1)                          )stage++;break;
			case 55: if (ralff.setHand(false, true)              )stage++;break;
			case 56: if (ralff.sleep(.5)                         )stage++;break;
			case 57: if (ralff.drivePID(1,1, 1)                  )stage++;break;
			case 58:                                              stage++;break;
			case 59: if (ralff.setForward(true)                  )stage++;break;
			case 60: if (ralff.moveArm(Arm.Mode.kTransport,false))stage++;break;
			default: break;
		}
	}

	public void periodicPowerControl() {
		switch (stage) {
			case  0: if (ralff.init()                            )stage++;break;
			case  1: if (ralff.shiftLow()                        )stage++;break;
			case  2: if (ralff.setHand(false, false)             )stage++;break;
			case  3: if (ralff.setForward(false)                 )stage++;break;
			case  4: if (ralff.calibrateArm()                    )stage++;break;
			// Score ////////////////////////////////////////////)stage++;break;
			case  5: if (tubes >= 1                              )stage++;break;
			case  6: if (ralff.setForward(false)                 )stage++;break;
			case  7: Control.armMiddle = false                   ;stage++;break;
			case  8: if (ralff.moveArm(Arm.Mode.kTop,false)      )stage++;break;
			case  9: if (ralff.driveDistancePower(-11.25,-11, .7))stage++;break;
			case 10: if (ralff.moveArm(Arm.Mode.kTop,tWait)      )stage++;break;
			case 11: if (ralff.driveDistancePower(-5.2,-5.2, .5) )stage++;break;
			case 12: if (ralff.moveArm(null,false)               )stage++;break;
			case 13: if (ralff.sleep(1)                          )stage++;break;
			case 14: if (ralff.setHand(false, true)              )stage++;break;
			case 15: if (ralff.sleep(.5)                         )stage++;break;
			case 16: if (ralff.driveDistancePower(2,2, .5)       )stage++;break;
			case 17:                                              stage++;break;
			case 18: if (ralff.setForward(true)                  )stage++;break;
			case 19: if (ralff.moveArm(Arm.Mode.kTransport,false))stage++;break;
			// Pick up second tube //////////////////////////////)stage++;break;
			case 20: if (tubes >= 2                              )stage++;break;
			case 21: Control.armMiddle=true                      ;stage++;break;
			case 22: if (ralff.moveArm(Arm.Mode.kLoadFloor,fWait))stage++;break;
			case 23:                                              stage++;break;
			case 24: if (ralff.driveDistancePower(4.7,6, .42,.7) )stage++;break;
			case 25: if (ralff.setHand(false, false)             )stage++;break;
			case 26: if (ralff.moveArm(Arm.Mode.kTransport,false))stage++;break;
			// Score second tube ////////////////////////////////)stage++;break;
			case 27: if (ralff.setForward(false)                 )stage++;break;
			case 28: if (ralff.moveArm(Arm.Mode.kTop,false)      )stage++;break;
			case 29: if (ralff.driveDistancePower(0,-1.5, .5)    )stage++;break;
			case 30: if (ralff.driveDistancePower(-4,-4, .7)     )stage++;break;
			case 31: if (ralff.moveArm(Arm.Mode.kTop,tWait)      )stage++;break;
			case 32: if (ralff.driveDistancePower(-2.7,-2.7, .5) )stage++;break;
			case 33: if (ralff.moveArm(null,false)               )stage++;break;
			case 34: if (ralff.sleep(1)                          )stage++;break;
			case 35: if (ralff.setHand(false, true)              )stage++;break;
			case 36: if (ralff.sleep(.5)                         )stage++;break;
			case 37: if (ralff.driveDistancePower(2,2, .5)       )stage++;break;
			case 38:                                              stage++;break;
			case 39: if (ralff.setForward(true)                  )stage++;break;
			case 40: if (ralff.moveArm(Arm.Mode.kTransport,false))stage++;break;
			// Pick up third tube ///////////////////////////////)stage++;break;
			case 41: if (tubes >= 3                              )stage++;break;
			case 42: if (ralff.moveArm(Arm.Mode.kLoadFloor,fWait))stage++;break;
			case 43:                                              stage++;break;
			case 44: if (ralff.driveDistancePower(5,7, .36,.6)   )stage++;break;
			case 45: if (ralff.setHand(false, false)             )stage++;break;
			case 46: if (ralff.moveArm(Arm.Mode.kTransport,false))stage++;break;
			// Score third tube /////////////////////////////////)stage++;break;
			case 47: if (ralff.setForward(false)                 )stage++;break;
			case 48: if (ralff.moveArm(Arm.Mode.kTop,false)      )stage++;break;
			case 49: if (ralff.driveDistancePower(0,-1.7, .5)    )stage++;break;
			case 50: if (ralff.driveDistancePower(-4,-4.5, .6)   )stage++;break;
			case 51: if (ralff.moveArm(Arm.Mode.kTop,tWait)      )stage++;break;
			case 52: if (ralff.driveDistancePower(-2,-2, .5)     )stage++;break;
			case 53: if (ralff.moveArm(null,false)               )stage++;break;
			case 54: if (ralff.sleep(1)                          )stage++;break;
			case 55: if (ralff.setHand(false, true)              )stage++;break;
			case 56: if (ralff.sleep(.5)                         )stage++;break;
			case 57: if (ralff.driveDistancePower(1,1, .5)       )stage++;break;
			case 58:                                              stage++;break;
			case 59: if (ralff.setForward(true)                  )stage++;break;
			case 60: if (ralff.moveArm(Arm.Mode.kTransport,false))stage++;break;
			default: break;
		}
	}
}
