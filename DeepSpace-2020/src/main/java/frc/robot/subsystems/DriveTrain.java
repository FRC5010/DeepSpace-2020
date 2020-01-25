/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.TeleopDefault;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  WPI_TalonSRX rightMotors = RobotMap.rightMotor1;
  WPI_TalonSRX leftMotors = RobotMap.leftMotor1;

  @Override
  public void initDefaultCommand() {
    // Set a default command for a subsystem here.
    // setDefaultCommand(new TeleopDefault());
    setDefaultCommand(new TeleopDefault());
  }

  public void stop(){
    leftMotors.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
    rightMotors.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
  }
  
  public void drive(double leftPower, double rightPower){
    leftMotors.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, leftPower);
    rightMotors.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, rightPower);
    SmartDashboard.putNumber("Left Voltage", leftMotors.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightMotors.getMotorOutputPercent());
  }
}


