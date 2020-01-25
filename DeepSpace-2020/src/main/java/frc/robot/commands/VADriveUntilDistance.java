/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.DirectionSensor;
import frc.robot.subsystems.Pose;
import frc.robot.subsystems.VisionAssistedDrive;

public class VADriveUntilDistance extends Command {
  VisionAssistedDrive vad;

  private double lastHeadingError = 0;
  private double lastError = 0;
  private double setpoint = 0;
  private double prevError = 0;
  private int timesAtPrevError = 0;

  public VADriveUntilDistance (double setpoint) {
    this.setpoint = setpoint;
    vad = RobotMap.visionDrive;
    vad.printPIDValues();
    requires(RobotMap.driveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		SmartDashboard.putString("Command", this.getClass().getSimpleName());
    SmartDashboard.putString(this.getClass().getSimpleName(), "init");
    lastError = 0;
    lastHeadingError = 0;
    prevError = 0;
    timesAtPrevError = 0;
    moveTowardsTarget(setpoint, lastError);
    SmartDashboard.putBoolean("Vision Interrupted", false);
    SmartDashboard.putBoolean("Vision Ended", false);
    SmartDashboard.putBoolean("Vision Finished", false);
  }
 
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putString(this.getClass().getSimpleName(), "working");

    double output = Math.min(moveTowardsTarget(setpoint, lastError), 0.5);
    double turn = turnTowards(0, lastHeadingError);

    RobotMap.driveTrain.drive(output - turn, output + turn);

    SmartDashboard.putNumber("Move error", lastError);
    SmartDashboard.putNumber("Steer error", lastHeadingError);
  }

  public double moveTowardsTarget(double setpoint, double lastError) {
    double moveAmt = 0;
    if (Pose.getCurrentPose().limeLight.tValid) {
      double distance = Pose.getCurrentPose().limeLight.tDistance;
      double error =  distance - setpoint;
      double errorDelta = error - lastError;

      moveAmt = vad.getMoveKp() * error + vad.getMoveKd() * errorDelta;

      double moveMin = vad.getMoveMin();
      moveAmt = Math.max(moveMin, Math.abs(moveAmt)) * Math.signum(moveAmt);
      this.lastError = error;
    }
    SmartDashboard.putNumber(this.getClass().getSimpleName() + " moveAmt", moveAmt);
    return moveAmt;
  }

  public double turnTowards(double desiredHeading, double lastHeadingError) {
    double turnAmt = 0;
    if (Pose.getCurrentPose().limeLight.tValid) {
      double heading = Pose.getCurrentPose().limeLight.tX;
      double headingError = DirectionSensor.boundHalfDegrees(desiredHeading - heading);
      double headingDelta = headingError - lastHeadingError;

      turnAmt = vad.getSteerKp() * headingError + vad.getSteerKd() * headingDelta;

      double steerMin = vad.getSteerMin();
      turnAmt = Math.max(steerMin, Math.abs(turnAmt)) * Math.signum(turnAmt);
      this.lastHeadingError = headingError;
    }
    SmartDashboard.putNumber(this.getClass().getSimpleName() + " Steer", turnAmt);
    return turnAmt;
  }


  // Make this return true when this Command no longer needs to run execute()

  @Override
  protected boolean isFinished() {
    double manualOverride = Robot.oi.driveTrainForward.getValue();
    double steerOverride = Robot.oi.driveTrainTurn.getValue();
    if (((int) lastError) / 1 == ((int) prevError) / 1 ) {
      timesAtPrevError++;
    } else {
      timesAtPrevError = 0;
    }
    prevError = lastError;
    boolean finished = manualOverride != 0 || steerOverride != 0 ||  (Math.abs(lastHeadingError) < 1 && Math.abs(lastError) < 1) || timesAtPrevError > 20;
    SmartDashboard.putBoolean("Vision Finished", finished);
    return finished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putBoolean("Vision Ended", true);
    SmartDashboard.putString(this.getClass().getSimpleName(), "end");
    RobotMap.driveTrain.stop();
    lastError = 0;
    lastHeadingError = 0;
    prevError = 0;
    timesAtPrevError = 0;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    SmartDashboard.putBoolean("Vision Interrupted", true);
    end();
  }
}
