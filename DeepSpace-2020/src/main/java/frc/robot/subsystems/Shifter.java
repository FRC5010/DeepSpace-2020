/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Shifter extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static boolean isLowGear = false;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void shiftUp(){
    RobotMap.shiftSolenoid.set(false);
    isLowGear = false;
    SmartDashboard.putBoolean("Is Low gear", isLowGear);
  }

  public void shiftDown(){
    RobotMap.shiftSolenoid.set(true);
    isLowGear = true;
    SmartDashboard.putBoolean("Is Low gear", isLowGear);
  }
}
