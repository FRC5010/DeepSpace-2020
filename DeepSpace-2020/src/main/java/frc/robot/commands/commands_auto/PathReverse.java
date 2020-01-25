/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.commands_auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class PathReverse extends Command {
  private static final double max_velocity = 17.89;
  Trajectory trajectory;
  EncoderFollower left, right;

  public PathReverse(Trajectory lTraj, Trajectory rTraj) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    
    left = new EncoderFollower(lTraj);
		right = new EncoderFollower(rTraj);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SmartDashboard.putBoolean("Running", true);

		// Encoder Position is the current, cumulative position of your encoder. If
		// you're using an SRX, this will be the
		// 'getEncPosition' function.
		// 1000 is the amount of encoder ticks per full revolution
		// Wheel Diameter is the diameter of your wheels (or pulley for a track system)
		// in meters
		RobotMap.leftEncoder.reset();
		RobotMap.rightEncoder.reset();
		
		//TODO: Eventually needs to be flipped after pathweaver update.
		left.configureEncoder(RobotMap.distance.getLeftRaw(),RobotMap.encoderPPR, .5);
		right.configureEncoder(RobotMap.distance.getRightRaw(),RobotMap.encoderPPR, .5);

		// The first argument is the proportional gain. Usually this will be quite high
		// The second argument is the integral gain. This is unused for motion profiling
		// The third argument is the derivative gain. Tweak this if you are unhappy with
		// the tracking of the trajectory
		// The fourth argument is the velocity ratio. This is 1 over the maximum
		// velocity you provided in the
		// trajectory configuration (it translates m/s to a -1 to 1 scale that your
		// motors can read)
		// The fifth argument is your acceleration gain. Tweak this if you want to get
		// to a higher or lower speed quicker
		left.configurePIDVA(1.0, 0.0, 0.0, 1 / max_velocity, 0);
		right.configurePIDVA(1.0, 0, 0, 1 / max_velocity, 0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
	Trajectory.Segment seg = left.getSegment();
	Trajectory.Segment segR = right.getSegment();
    SmartDashboard.putNumber("left accel", seg.acceleration);
			SmartDashboard.putNumber("left time delta", seg.dt);
			SmartDashboard.putNumber("left heading", seg.heading);
			SmartDashboard.putNumber("left jerk", seg.jerk);
			SmartDashboard.putNumber("left pos", seg.position);
			SmartDashboard.putNumber("left velocity", seg.velocity);
			SmartDashboard.putNumber("left x", seg.x);
			SmartDashboard.putNumber("left y", seg.y);
			
			SmartDashboard.putNumber("right accel", segR.acceleration);
			SmartDashboard.putNumber("right time delta", segR.dt);
			SmartDashboard.putNumber("right heading", segR.heading);
			SmartDashboard.putNumber("right jerk", segR.jerk);
			SmartDashboard.putNumber("right pos", segR.position);
			SmartDashboard.putNumber("right velocity", segR.velocity);
			SmartDashboard.putNumber("right x", segR.x);
			SmartDashboard.putNumber("right y", segR.y);
			 double distance_covered = ((double)(RobotMap.distance.getLeftRaw() - 0) / RobotMap.encoderPPR)
		                * .5;
			SmartDashboard.putNumber("distance covered", distance_covered);
		double l = left.calculate(-RobotMap.distance.getLeftRaw());
		
		double r = right.calculate(-RobotMap.distance.getRightRaw());

		double gyro_heading = -(RobotMap.direction.angle());// Assuming the gyro is giving a value in degrees
		//SmartDashboard.putNumber("gyro heading", gyro_heading);
		
		//TODO: Negate gyro heading after the update if needed.
		double desired_heading = Pathfinder.r2d(seg.heading); // Should also be in degree
		SmartDashboard.putNumber("desired Heading", desired_heading);
		

		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
		SmartDashboard.putNumber("angle difference", angleDifference);
		double turn = 0.8 * (-1.0 / 80.0) * angleDifference;
		 //turn = 0;

		SmartDashboard.putNumber("left output", -(l + turn));
		SmartDashboard.putNumber("right output", -(r - turn));
		SmartDashboard.putNumber("turn ", turn);
		RobotMap.driveTrain.drive(-(l + turn), -(r - turn));

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return left.isFinished() || right.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
