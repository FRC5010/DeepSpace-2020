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

public class RaiseElevator extends Command {
  private double moveUp;

  public RaiseElevator() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(RobotMap.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		SmartDashboard.putString("Command", this.getClass().getSimpleName());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    moveUp = Robot.oi.elevatorLiftControl.getValue();

    // Read and output this to get screen feedback
    RobotMap.elevator.isCargoGamePiece = Robot.oi.elevatorGamePieceSelector.get();
    SmartDashboard.putBoolean("Elevator MM Cargo GP", RobotMap.elevator.isCargoGamePiece);

    RobotMap.elevator.raiseElevator(moveUp);
    SmartDashboard.putNumber("Elevator power", moveUp);
    SmartDashboard.putNumber("Elevator position", RobotMap.elevatorMotor.getSelectedSensorPosition());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
