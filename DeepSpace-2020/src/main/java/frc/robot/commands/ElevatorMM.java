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
import frc.robot.Robot;
import frc.robot.subsystems.Elevator;

public class ElevatorMM extends Command {
  double setPoint = 0;
  Elevator.Position position = Elevator.Position.LOW;

  public ElevatorMM(Elevator.Position position) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.position = position;
    requires(RobotMap.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		SmartDashboard.putString("Command", this.getClass().getSimpleName());
    if (RobotMap.elevator.isCargoGamePiece) {
      switch (position) {
      case LOW: {
        setPoint =250;
        break;
      }
      case MIDDLE: {
        setPoint = Elevator.CARGO_MIDDLE;
        break;
      }
      case HIGH: {
        setPoint = Elevator.CARGO_HIGH;
        break;
      }
      case SHIP: {
        setPoint = Elevator.CARGO_SHIP;
      }
      }
    } else {
      switch (position) {
      case LOW: {
        setPoint = Elevator.HATCH_LOW;
        break;
      }
      case MIDDLE: {
        setPoint = Elevator.HATCH_MIDDLE;
        break;
      }
      case HIGH: {
        setPoint = Elevator.HATCH_HIGH;
        break;
      }
      case SHIP: {
        setPoint = Elevator.ZERO;
      }
      }
    }
    RobotMap.elevator.moveToPosition(setPoint);
    SmartDashboard.putString("Elevator MM", "Initialized");
    SmartDashboard.putString("Elevator MM Position", position.toString());
    SmartDashboard.putNumber("Elevator MM Setpoint", setPoint);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putString("ElevatorMM", "Running");
    SmartDashboard.putNumber("Elevator position", RobotMap.elevatorMotor.getSelectedSensorPosition());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double manualPower = Robot.oi.elevatorLiftControl.getValue();
    double pos = RobotMap.elevator.getCurrentPosition();
    double err = Math.abs(setPoint - pos);
    SmartDashboard.putNumber("Elevator MM err", err);
    return err < 50 // this means we're close enough to the target
      || RobotMap.elevator.isSomethingStuck(RobotMap.elevatorMotor.getMotorOutputPercent()) 
      || 0 != manualPower; // Moving the joystick will abort MM
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putString("ElevatorMM", "Done");
    RobotMap.elevator.returnToManualControl();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

}
