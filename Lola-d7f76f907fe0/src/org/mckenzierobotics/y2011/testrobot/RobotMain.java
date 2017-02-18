/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.mckenzierobotics.y2011.testrobot;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Watchdog;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotMain extends IterativeRobot {

	private final Robot robot = new Robot();

	private final Controller teleop = new Teleop();
	private final Controller auto   = new Autonomous();
	private final Controller disable = new Disabled();

	private Controller controller = disable;

	public static Watchdog dog = edu.wpi.first.wpilibj.Watchdog.getInstance();

	public void robotInit() {
		controller = disable;
    }

	public void autonomousInit() {
		controller = auto;
		controller.init();
	}
	public void teleopInit() {
		controller = teleop;
		controller.init();
	}
	public void disabledInit() {
		controller = disable;
		controller.init();
	}

    public void autonomousPeriodic() {
		periodic();
    }
    public void teleopPeriodic() {
        periodic();
    }
	public void disabledPeriodic() {
		periodic();
	}

	public void periodic() {
		dog.feed();
		controller.periodic();
		dog.feed();
		robot.periodic();
		dog.feed();
		Control.lcd.update();
	}

    
}
