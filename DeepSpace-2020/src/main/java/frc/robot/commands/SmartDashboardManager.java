/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

/**
 * Add your docs here.
 */
public class SmartDashboardManager {

    static public double roundDouble(double number, double places){
         //The first number is the number that you want rounded, the second number is the number of digits you want to survive
        number = number * places;
        number = Math.floor(number);
        number = number / places;
        return number;  
    }

}
