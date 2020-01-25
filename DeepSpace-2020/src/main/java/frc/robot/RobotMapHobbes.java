/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;

/**
 * Add your docs here.
 */
public class RobotMapHobbes {
    public static void initHobbes() {
        RobotMap.initRobotComponents();
        RobotMap.rightMotor1 = new WPI_TalonSRX(4);
        RobotMap.rightMotor2 = new WPI_TalonSRX(5);
    
        RobotMap.leftMotor1 = new WPI_TalonSRX(1);
        RobotMap.leftMotor2 = new WPI_TalonSRX(2);
        
        RobotMap.rightMotor1.setInverted(true);
        RobotMap.rightMotor2.setInverted(true);
    
        RobotMap.rightMotor2.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 4);
    
        RobotMap.leftMotor2.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 1);
    
        RobotMap.initSubsystems();
      }    
}
