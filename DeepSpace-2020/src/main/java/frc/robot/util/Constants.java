/**
 * Simple class containing constants used throughout project
 */
package frc.robot.util;

import frc.robot.subsystems.Wrist;

public class Constants {
	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
	 * or 3. Only the first two (0,1) are visible in web-based configuration.
	 */
	public static final int kSlotIdx = 0;
	public static final int kSlotDown = 1;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
	 * we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 30;

	/**
	 * Motor neutral dead-band, set to the minimum 0.1%.
	 */
	public final static double kNeutralDeadband = 0.001;

	/* Current threshold to trigger current limit */
	public static final int kPeakCurrentAmps = 38;

	/* Duration after current exceed Peak Current to trigger current limit */
	public static final int kPeakTimeMs = 500;

	/* Current to mantain once current limit has been triggered */
	public static final int kContinCurrentAmps = 38;

	/**
	 * Gains used in Motion Magic, to be adjusted accordingly Gains(kp, ki, kd, kf,
	 * izone, peak output);
	 */
	public static final Gains kDriveGains = new Gains(0, 0.0, 0, 0, 0, 0);
	public static final Gains kGains = new Gains(4, 0.0, 8, 0.2775, 0, 1.0);

	public static final Gains wristGains = new Gains(3.5, 0, 0, 0.0, 0, Wrist.MAX_FWD_OUT);
	public static final Gains wristDownGains = new Gains(0.1, 0, 0, 0.0, 0, Wrist.MAX_REV_OUT);
}