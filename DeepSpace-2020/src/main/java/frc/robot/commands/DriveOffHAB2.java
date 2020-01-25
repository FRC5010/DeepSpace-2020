/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DirectionSensor;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.VisionAssistedDrive;

public class DriveOffHAB2 extends Command {
  int numberOfBumps;
  VisionAssistedDrive vad;
  double desiredHeading, speed;
  double lastHeadingError, prevError;
  double startTime;

  public DriveOffHAB2(double speed) {
    requires(RobotMap.driveTrain);
    requires(RobotMap.direction);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    vad = RobotMap.visionDrive;
    this.desiredHeading = 0;
    this.speed = speed;
    lastHeadingError = 0;
    prevError = 0;
    startTime = 0;
    numberOfBumps = 0;
    SmartDashboard.putNumber(this.getClass().getSimpleName() + " Speed", speed);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		SmartDashboard.putString("Command", this.getClass().getSimpleName());
    lastHeadingError = 0;
    prevError = 0;
    numberOfBumps = 0;
    startTime = RobotController.getFPGATime() / 1000000.0;
    speed = SmartDashboard.getNumber(this.getClass().getSimpleName() + " Speed", speed);
  }

  public double turnTowards() {
    double turnAmt = 0;
    double heading = RobotMap.gyro.getAngle();
    double headingError = DirectionSensor.boundHalfDegrees(desiredHeading - heading);
    double headingDelta = headingError - lastHeadingError;
    double currentTime = RobotController.getFPGATime() / 1000000.0;
    double timeDelta = currentTime - startTime;

    turnAmt = vad.getSteerKp() * headingError + (vad.getSteerKd() * headingDelta / timeDelta);

    double steerMin = vad.getSteerMin();
    turnAmt = Math.max(steerMin, Math.abs(turnAmt)) * Math.signum(turnAmt);
    this.lastHeadingError = headingError;
    return turnAmt;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (numberOfBumps == 0) {
      if (Pose.getCurrentPose().pitch < -15) {
        ++numberOfBumps;
      }
    } else if (numberOfBumps == 1) {
      if (Pose.getCurrentPose().pitch > -5) {
        ++numberOfBumps;
      }
    }
    SmartDashboard.putNumber(this.getClass().getSimpleName() + " Bumps", numberOfBumps);
    double turnAmt = turnTowards();
    RobotMap.driveTrain.drive(speed + turnAmt, speed - turnAmt);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double manualOverride = Math.abs(Robot.oi.driveTrainForward.getValue());
    boolean finished = (manualOverride > 0) || numberOfBumps > 1;
    return finished;  
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
    SmartDashboard.putString(this.getClass().getSimpleName(), "Interrupted");
  }
}
