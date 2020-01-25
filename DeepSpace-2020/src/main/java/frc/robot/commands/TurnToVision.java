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

public class TurnToVision extends Command {
  VisionAssistedDrive vad;
  double desiredHeading;
  double lastHeadingError, prevError, timesAtPrevError;
  double startTime;
  public TurnToVision() {
    requires(RobotMap.driveTrain);
    requires(RobotMap.direction);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    vad = RobotMap.visionDrive;
    desiredHeading = 0;
    lastHeadingError = 0;
    prevError = 0;
    timesAtPrevError = 0;
    startTime = 0;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		SmartDashboard.putString("Command", this.getClass().getSimpleName());
    lastHeadingError = 0;
    prevError = 0;
    timesAtPrevError = 0;
    startTime = RobotController.getFPGATime() / 1000000.0;
  }

  public double turnTowards() {
    double turnAmt = 0;
    if (Pose.getCurrentPose().limeLight.tValid) {
      double heading = RobotMap.vision.getX();
      double headingError = (desiredHeading - heading);
      double headingDelta = DirectionSensor.boundHalfDegrees(headingError - lastHeadingError);
      double currentTime = RobotController.getFPGATime() / 1000000.0;
      double timeDelta = currentTime - startTime;

      turnAmt = vad.getSteerKp(true) * headingError + (vad.getSteerKd(true) * headingDelta / timeDelta);

      double steerMin = vad.getSteerMin(true);
      turnAmt = Math.max(steerMin, Math.abs(turnAmt)) * Math.signum(turnAmt);
      this.lastHeadingError = headingError;
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " err", headingError);
    }
    SmartDashboard.putNumber(this.getClass().getSimpleName() + " Steer", turnAmt);
    return turnAmt;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double turnAmt = turnTowards();
    RobotMap.driveTrain.drive(-turnAmt, turnAmt);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double manualOverride = Robot.oi.driveTrainForward.getValue();
    double steerOverride = Robot.oi.driveTrainTurn.getValue();
    if ( ((int)lastHeadingError)/1 == ((int)prevError)/1 ) {
      timesAtPrevError++;
    } else {
      timesAtPrevError = 0;
    }
    prevError = lastHeadingError;
    boolean finished = (0 != manualOverride || 0 != steerOverride) ||  
      (Math.abs(lastHeadingError) < 1) || timesAtPrevError > 20;

    SmartDashboard.putString(this.getClass().getSimpleName(), finished ? "Finished" : "Running");
    return finished;  
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.driveTrain.stop();
    lastHeadingError = 0;
    prevError = 0;
    timesAtPrevError = 0;
    SmartDashboard.putString(this.getClass().getSimpleName(), "Ended");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
    SmartDashboard.putString(this.getClass().getSimpleName(), "Interrupted");
  }
}
