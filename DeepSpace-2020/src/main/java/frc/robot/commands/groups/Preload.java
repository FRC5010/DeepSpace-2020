/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.BeakClose;
import frc.robot.commands.PauseForTime;
import frc.robot.commands.PreloadFinish;
import frc.robot.commands.PreloadSetup;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.WristMM;
//import frc.robot.commands.spinWheels;
import frc.robot.subsystems.Wrist;

public class Preload extends CommandGroup {
  /**
   * Add your docs here.
   */

  public static boolean isPreloading = false;
  public Preload() {
    addSequential(new PreloadSetup());
    addSequential(new ResetGyro());
    addSequential(new WristMM(Wrist.Position.HIGH));
    addSequential(new PauseForTime(1000));
    addSequential(new BeakClose());
    addSequential(new PreloadFinish());
  }
}
