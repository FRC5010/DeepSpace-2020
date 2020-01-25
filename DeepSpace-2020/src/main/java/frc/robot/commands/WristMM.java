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
import frc.robot.subsystems.Wrist;
import frc.robot.Robot;

public class WristMM extends Command {
  private double setPoint = 0;
  private Wrist.Position position;

  public WristMM(Wrist.Position Position) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.position = Position;

    //System.out.println("Wrist Position: "+ position);
    requires(RobotMap.wrist);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
		SmartDashboard.putString("Command", this.getClass().getSimpleName());
    SmartDashboard.putString("Wrist MM", "Initialized");
    setPoint = Wrist.HATCH_LOW;
    switch (position) {
      case LOW: {
        if (RobotMap.elevator.isCargoGamePiece) {
          setPoint = Wrist.CARGO_MIDDLE;
        } else {
          if (Preload.isPreloading) {
            setPoint = Wrist.HATCH_LOW;
          }
        }
        break;
      }
      case MIDDLE: {
        if (RobotMap.elevator.isCargoGamePiece) {
          setPoint = Wrist.CARGO_MIDDLE;
        } else {
          if (Preload.isPreloading) {
            setPoint = Wrist.HATCH_MIDDLE;
          }
        }
        break;
      }
      case HIGH: {
        if (RobotMap.elevator.isCargoGamePiece) {
          setPoint = Wrist.CARGO_HIGH;
        } else {
          if (Preload.isPreloading) {
            setPoint = Wrist.HATCH_HIGH;
          }
        }
        break;
      }
      case SHIP: {
          setPoint = Wrist.CARGO_SHIP;
          break;
        }
      case PRELOAD: {
          setPoint = Wrist.PRELOAD;
          break;
       }
    }
    SmartDashboard.putNumber("Wrist MM Setpoint", setPoint);
    if (Wrist.lastMMPosition == Wrist.HATCH_LOW) {
      RobotMap.wrist.reset();
    }
    Wrist.lastMMPosition = setPoint;
    SmartDashboard.putString("Wrist MM Position", position.toString());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putString("Wrist MM", "Running");
    RobotMap.wrist.moveToPosition(setPoint);
  }

  // Make this return true when this Command no longer needs to run execute()
  double timesAtPrevError;

  @Override
  protected boolean isFinished() {
    double err = Math.abs(setPoint -
      RobotMap.wrist.ticsToAngle(RobotMap.wrist.getCurrentPosition()));
    double manualPower = Robot.oi.wristControl.getValue();
    // if it gets stuck down make that sensor value 0 or if its stuck up make it go
    // down
    boolean isStuck = RobotMap.wrist.isSomethingStuck(RobotMap.wristMotor.getMotorOutputPercent());
    if (err < 5 || (isStuck && Wrist.lastMMPosition == Wrist.HATCH_LOW)) {
      RobotMap.wristMotor.setSelectedSensorPosition((int)RobotMap.wrist.angleToTics(setPoint));
    }
    return manualPower != 0 // moving the joystick will abort MM
      || isStuck || err < 5;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SmartDashboard.putString("Wrist MM", "Done");
    RobotMap.wrist.returnToManualControl();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
