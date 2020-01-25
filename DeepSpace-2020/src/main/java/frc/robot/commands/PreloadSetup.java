/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.groups.Preload;
import frc.robot.subsystems.Vision.CamMode;
import frc.robot.subsystems.Vision.LEDMode;
import frc.robot.subsystems.Vision.Stream;
import frc.robot.subsystems.Wrist;

public class PreloadSetup extends Command {
  public PreloadSetup() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		SmartDashboard.putString("Command", this.getClass().getSimpleName());
    RobotMap.vision.setLimeLightLEDMode(LEDMode.ON);
    RobotMap.vision.setCamMode(CamMode.DRIVER);
    RobotMap.vision.setStreaming(Stream.SIDE_BY_SIDE);
    RobotMap.wristMotor.setSelectedSensorPosition(RobotMap.wrist.angleToTics(Wrist.PRELOAD));
    Preload.isPreloading = true;
    Wrist.lastMMPosition = Wrist.PRELOAD;
    BallControl.gripBall = true;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
