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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;

public class PathForward extends Command {
  private static final double max_velocity = 17.89;
  Trajectory trajectory;
  EncoderFollower left, right;

  public PathForward(Trajectory lTraj, Trajectory rTraj) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
	
	//Swapped because of Pathweaver issues for the time being.
	//TODO: REVERSED WHEN FIXED!!!!!!!!!!!!
    	left = new EncoderFollower(rTraj);
		right = new EncoderFollower(lTraj);
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
    SmartDashboard.putNumber("left accel", seg.acceleration);
			SmartDashboard.putNumber("left time delta", seg.dt);
			//TODO: REVERSE NEGATION AFTER PATHWEAVER FIX.
			SmartDashboard.putNumber("left heading", -Pathfinder.r2d(seg.heading));
			SmartDashboard.putNumber("left jerk", seg.jerk);
			SmartDashboard.putNumber("left pos", seg.position);
			SmartDashboard.putNumber("left velocity", seg.velocity);
			SmartDashboard.putNumber("left x", seg.x);
			SmartDashboard.putNumber("left y", seg.y);
			
			SmartDashboard.putNumber("right accel", right.getSegment().acceleration);
			//TODO: REVERSE NEGATION AFTER PATHWEAVER FIX.
			SmartDashboard.putNumber("right time delta", -Pathfinder.r2d(right.getSegment().dt));
			SmartDashboard.putNumber("right heading", right.getSegment().heading);
			SmartDashboard.putNumber("right jerk", right.getSegment().jerk);
			SmartDashboard.putNumber("right pos", right.getSegment().position);
			SmartDashboard.putNumber("right velocity", right.getSegment().velocity);
			SmartDashboard.putNumber("right x", right.getSegment().x);
			SmartDashboard.putNumber("right y", right.getSegment().y);
			 double distance_covered = ((double)(RobotMap.distance.getLeftRaw() - 0) / RobotMap.encoderPPR)
		                * .5;
			SmartDashboard.putNumber("distance covered", distance_covered);
		//Took out negation.
		double l = left.calculate(RobotMap.distance.getLeftRaw());
		
		//Values negated here so it works.
		double r = right.calculate(RobotMap.distance.getRightRaw());
		
		double gyro_heading = RobotMap.direction.angle();// Assuming the gyro is giving a value in degrees
		
		//TODO: REVERSE NEGATION AFTER PATHWEAVER FIX!!!!!
		double desired_heading = -Pathfinder.r2d(seg.heading); // Should also be in degrees
		//TODO: REVERSE NEGATION AFTER PATHWEAVER UPDATE
		SmartDashboard.putNumber("desired Heading", desired_heading);

		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
		SmartDashboard.putNumber("angle difference", angleDifference);
		double turn = 0.8 * (-1.0 / 80.0) * angleDifference;
		 //turn = 0;

		SmartDashboard.putNumber("left output", (l + turn));
		SmartDashboard.putNumber("right output", (r - turn));
		SmartDashboard.putNumber("turn ", turn);
		RobotMap.driveTrain.drive((l + turn), (r - turn));

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return left.isFinished() || right.isFinished();
  }

  // Called once after isFinished retu0rns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
