/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.commands_auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotMap_Paths;
import frc.robot.RobotMap_Paths.MotionProfiles;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class PathFollower5010 extends Command {

	EncoderFollower5010 left, right;
	public enum Direction { kForward, kRevNormal, kRevFlipped }
	private Direction isFwd = Direction.kForward;
	String path; 
	public PathFollower5010(MotionProfiles trajectory, Direction isFwd) {
		path = trajectory.name();
		load(RobotMap_Paths.leftTrajectories.get(trajectory), RobotMap_Paths.rightTrajectories.get(trajectory), isFwd);
	}
	
	public PathFollower5010(Trajectory lTraj, Trajectory rTraj, Direction isFwd) {
		load(lTraj, rTraj, isFwd);
	}

	public void load(Trajectory lTraj, Trajectory rTraj, Direction isFwd) {
		requires(RobotMap.driveTrain);
		requires(RobotMap.direction);
		requires(RobotMap.distance);
		this.isFwd = isFwd;
		// Swapped because of Pathweaver issues for the time being.
		// TODO: REVERSED WHEN FIXED!!!!!!!!!!!!
		// The isRight parameter can be determined relative to the isFwd parameter
		if(isFwd == Direction.kForward) {
			left = new EncoderFollower5010(rTraj, false, isFwd);
			right = new EncoderFollower5010(lTraj, true, isFwd);
		} else {
			left = new EncoderFollower5010(lTraj, false, isFwd);
			right = new EncoderFollower5010(rTraj, true, isFwd);
		}
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		SmartDashboard.putString("Command", path);

		// Encoder Position is the current, cumulative position of your encoder. If
		// you're using an SRX, this will be the
		// 'getEncPosition' function.
		// Second param is the amount of encoder ticks per full revolution
		// Wheel Diameter is the diameter of your wheels (or pulley for a track system)
		// in the chosen units
		left.reset();
		right.reset();
		RobotMap.distance.reset();
		left.configureEncoder(RobotMap.leftEncoder, RobotMap.direction, RobotMap.encoderPPR, RobotMap_Paths.wheel_diameter);
		right.configureEncoder(RobotMap.rightEncoder, RobotMap.direction, RobotMap.encoderPPR, RobotMap_Paths.wheel_diameter);

		// The first argument is the proportional gain. Usually this will be quite high
		double kp = 0.0;
		// The second argument is the integral gain. This is unused for motion profiling
		// The third argument is the derivative gain. Tweak this if you are unhappy with
		// the tracking of the trajectory
		double kd = 0.00;
		// The fourth argument is the velocity ratio. This is 1 over the maximum
		// velocity you provided in the trajectory configuration (it translates m/s to a
		// -1 to 1 scale that your
		// motors can read)
		double Lkv;
		double Rkv;
		double Lka;
		double Rka;
		double Rvint;
		double Lvint;
		if(isFwd == Direction.kForward){
				Lkv =0.0434;
				Rkv =0.0421 ;
				Lka = 0.0005102038686;
				Rka = 0.008398880472;
				Rvint = 0.0935;
				Lvint = 0.0829;
		}else{
				 Lkv = -0.0434;
				 Rkv = -0.0421;
				 Lka =-0.002955356636;
				 Rka = -0.006656921477 ;
				 Rvint = -0.0935;
				Lvint = -0.0829;
		}
			
		// The fifth argument is your acceleration gain. Tweak this if you want to get
		// to a higher or lower speed quicker
	
		// Sixth - The position error tolerance to achieve before isFinished will return
		// true
		double ket = .1;
		// Seventh - The heading error tolerance to achieve before isFinished will
		// return true
		double kht = 1;
		left.configurePIDVA(kp, 0.0, kd, Lkv, Lka, ket, kht, Lvint);
		right.configurePIDVA(kp, 0.0, kd, Rkv, Rka, ket, kht, Rvint);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
  protected void execute() {
	double l = left.calculate();
	double r = right.calculate();
		
	Trajectory.Segment lseg = left.getSegment();
	Trajectory.Segment rseg = right.getSegment();
    SmartDashboard.putNumber("left accel", lseg.acceleration);
	SmartDashboard.putNumber("left time delta", lseg.dt);
	//TODO: REVERSE NEGATION AFTER PATHWEAVER FIX.
	SmartDashboard.putNumber("left heading", -Pathfinder.r2d(lseg.heading));
	SmartDashboard.putNumber("left jerk", lseg.jerk);
	SmartDashboard.putNumber("left pos", lseg.position);
	SmartDashboard.putNumber("left velocity", lseg.velocity);
	SmartDashboard.putNumber("left x", lseg.x);
	SmartDashboard.putNumber("left y", lseg.y);
		
	SmartDashboard.putNumber("right accel", rseg.acceleration);
	//TODO: REVERSE NEGATION AFTER PATHWEAVER FIX.
	SmartDashboard.putNumber("right time delta", rseg.dt);
	SmartDashboard.putNumber("right heading", rseg.heading);
	SmartDashboard.putNumber("right jerk", rseg.jerk);
	SmartDashboard.putNumber("right pos", rseg.position);
	SmartDashboard.putNumber("right velocity", rseg.velocity);
	SmartDashboard.putNumber("right x", rseg.x);
	SmartDashboard.putNumber("right y", rseg.y);
	double distance_covered = ((double)(RobotMap.distance.getLeftRaw() - 0) / RobotMap.encoderPPR) * RobotMap_Paths.wheel_diameter;
	SmartDashboard.putNumber("distance covered", distance_covered);

	//TODO: REVERSE NEGATION AFTER PATHWEAVER FIX!!!!!
	double desired_heading = -Pathfinder.r2d(lseg.heading); // Should also be in degrees
	//TODO: REVERSE NEGATION AFTER PATHWEAVER UPDATE
	SmartDashboard.putNumber("desired Heading", desired_heading);

	SmartDashboard.putNumber("left output", l);
	SmartDashboard.putNumber("right output", r);
	RobotMap.driveTrain.drive(l, r);
  }

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return left.isFinished() || right.isFinished() 
		|| Robot.oi.driveTrainForward.getValue() != 0
		|| Robot.oi.driveTrainTurn.getValue() != 0;
	}

	// Called once after isFinished retu0rns true
	@Override
	protected void end() {
		// if(isFinished()) {
		// 	if (left.isFinished())
		// 		System.out.println("LEFT THOUGHT I WAS FINISHED");
		// 	if (right.isFinished())
		// 		System.out.println("RIGHT THOUGHT I WAS FINISHED");
		// 	if(Robot.oi.driveTrainForward.getValue() != 0)
		// 		System.out.println("DRIVER INTERRUPTED ON THROTTLE");
		// 	if(Robot.oi.driveTrainTurn.getValue() != 0)
		// 		System.out.println("DRIVER INTERRUPTED ON STEER");
		// } else {
		// 	System.out.println("I DON'T KNOW WHY I'M ENDING");
		// }
		left.end();
		right.end();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		System.out.println("I WAS INTERRUPTED!");
	}
}
