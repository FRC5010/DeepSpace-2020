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
import frc.robot.RobotMap;
import frc.robot.Robot;
import frc.robot.subsystems.DirectionSensor;
import frc.robot.subsystems.VisionAssistedDrive;

public class TurnToAnAngle extends Command {
  VisionAssistedDrive vad;
  double desiredHeading;
  double lastHeadingError, prevError, timesAtPrevError;
  double startTime;

  public TurnToAnAngle(double desiredHeading) {
    requires(RobotMap.driveTrain);
    requires(RobotMap.direction);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    vad = RobotMap.visionDrive;
    this.desiredHeading = desiredHeading;
    lastHeadingError = 0;
    prevError = 0;
    timesAtPrevError = 0;
    startTime = 0;
    vad.printPIDValues();
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
    double heading = RobotMap.gyro.getAngle();
    double headingError = DirectionSensor.boundHalfDegrees(desiredHeading - heading);
    double headingDelta = headingError - lastHeadingError;
    double currentTime = RobotController.getFPGATime() / 1000000.0;
    double timeDelta = currentTime - startTime;

    turnAmt = vad.getSteerKp(true) * headingError + (vad.getSteerKd(true) * headingDelta / timeDelta);

    double steerMin = vad.getSteerMin(true);
    turnAmt = Math.max(steerMin, Math.abs(turnAmt)) * Math.signum(turnAmt);
    this.lastHeadingError = headingError;
    return turnAmt;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double turnAmt = turnTowards();
    RobotMap.driveTrain.drive(turnAmt, -turnAmt);
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

    return finished;  
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.driveTrain.stop();
    lastHeadingError = 0;
    prevError = 0;
    timesAtPrevError = 0;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
