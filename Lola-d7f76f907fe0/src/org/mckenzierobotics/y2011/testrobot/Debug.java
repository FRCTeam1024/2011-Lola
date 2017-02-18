package org.mckenzierobotics.y2011.testrobot;

/**
 *
 * @author LukeShu
 */
public class Debug {
	public static void println(String str) {
		System.out.println(str);
	}

	public static void err(Exception e) {
		Control.lcd.lines[0] = "ERROR:"+e.getMessage();
		System.err.println("Exception: "+e.getMessage());
		// Still print to System.out, because I'm not sure where System.err
		// shows up at (hopefully the driver station, but idk).
		System.out.println("Exception: "+e.getMessage());
	}
}
