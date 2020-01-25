/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.LimeLightState;
import frc.robot.commands.ShiftDown;
import frc.robot.commands.ShiftUp;
import frc.robot.commands.VADriveUntilDistance;
import frc.robot.commands.LimeLightState.State;

public class LowGearVAD extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LowGearVAD() {
    addSequential(new LimeLightState(State.AUTO));
    addSequential(new ShiftDown());
    addSequential(new VADriveUntilDistance(30));
    addSequential(new ShiftUp());
    addParallel(new LimeLightState(State.DRIVER));
  }
}
