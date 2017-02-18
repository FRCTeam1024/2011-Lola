package org.mckenzierobotics.y2011.testrobot;

/**
 *
 * @author LukeShu
 */
public class Func {
	public static int sign(int i) {
		return i/Math.abs(i);
	}

	public static int sign(double i) {
		return (int)(i/Math.abs(i));
	}
	
	public static double nearest(double setpoint, double a, double b) {
		double diff_a = Math.abs(setpoint-a);
		double diff_b = Math.abs(setpoint-b);
		if (diff_a<diff_b) {
			return a;
		} else {
			return b;
		}
	}

	public static int nearest(double x, double[] arr) {
		double oldDiff = Double.POSITIVE_INFINITY;
		int j = -1;
		for (int i=0; i<arr.length; i++) {
			double diff = x-arr[i];
			if (diff<oldDiff) {
				j = i;
				oldDiff = diff;
			}
		}
		return j;
	}

}
