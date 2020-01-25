/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDCommand;

public class SetHeight extends PIDCommand {
  double height;
  double lastHeight = 0;
  byte immobileCount = 0;
  private PIDController PID;
  double hatchHeight = 48;
  //Ground to hatch hole center
  double cargoHeightOffset = 22;
  //Hatch hole center to corresponding cargo hole height-wise
  double repeatHeight = 70;
  //Hatch hole center to next hatch hole center (height)

  public SetHeight(double inputHeight) {
    super(0.2,0.0,0.0);
    requires(RobotMap.elevator);
    height = inputHeight;

    PID = getPIDController();
    PID.setAbsoluteTolerance(5);
    PID.setOutputRange(-1,1);
    PID.reset();
  }

  private double snapToHeight() {
    //Returns height of what the bot is closest to height-wise
    //Locations (heights in cm):
    //48, 70, 118, 140, 188, 210
    //Alternates hatch locations and rocket cargo holes

    //Hatch holes
    //48, 118, 188

    //Cargo holes
    //70, 140, 210
    double snapHeight = 0;
    
    //double curHeight = RobotMap.elevator.getHeight();
    double curHeight = 48;

    if (curHeight < hatchHeight) {
      snapHeight = hatchHeight;
    }
    if (curHeight > repeatHeight * 3) {
      snapHeight = repeatHeight * 3;
    }
    for (int i = 0; i < 3; i++) {
      if (curHeight >= repeatHeight * i + hatchHeight/2 && curHeight < repeatHeight * i + hatchHeight + cargoHeightOffset/2) {
        snapHeight = repeatHeight * i + hatchHeight;
      }
      if (curHeight >= repeatHeight * (i + 1) - cargoHeightOffset/2 && curHeight < repeatHeight * (i + 1) + hatchHeight/2) {
        snapHeight = repeatHeight * (i + 1);
      }
    }
    return snapHeight;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      PID.setSetpoint(height);
      PID.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double currentHeight = 0;
    //double currentHeight = RobotMap.elevator.getHeight();
    //To do: make a system that can get the height

    if (currentHeight == lastHeight) {
        immobileCount++;
    } else {
        immobileCount = 0;
    }
    if (immobileCount > 100) {
        System.out.println("Elevator not responding.");
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return PID.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    PID.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  protected double returnPIDInput() {
    //return RobotMap.elevator.getHeight();
    
    return height;
  }
  protected void usePIDOutput(double output) {
    //RobotMap.elevator.elevMotor.set(output);
  }
}
