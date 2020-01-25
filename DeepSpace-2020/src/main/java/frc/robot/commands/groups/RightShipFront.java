/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap_Paths.MotionProfiles;
import frc.robot.commands.LimeLightState;
import frc.robot.commands.ShiftDown;
import frc.robot.commands.ShiftUp;
import frc.robot.commands.VADriveUntilDistance;
import frc.robot.commands.LimeLightState.State;
import frc.robot.commands.commands_auto.PathFollower5010;
import frc.robot.commands.commands_auto.PathFollower5010.Direction;

public class RightShipFront extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RightShipFront() {
    addSequential(new Preload());
   // addSequential(new DriveOffHABLevel2Sequence());

    // Right start to Ship right-front bay then backup
    addSequential(new ShiftUp());
    addSequential(new PathFollower5010(MotionProfiles.HAB1, Direction.kForward));
    addSequential(new LimeLightState(State.AUTO));
    addSequential(new ShiftDown());
    addSequential(new VADriveUntilDistance(50));
    addSequential(new ShiftUp());
    addParallel(new LimeLightState(State.DRIVER));
    // addSequential(new PathFollower5010(MotionProfiles.backupShip1R, Direction.kRevNormal));
    // addSequential(new PathFollower5010(MotionProfiles.Ship1RtoRP, Direction.kForward));
    // addSequential(new VisionGrabHatch());
    //addSequential(new PathFollower5010(MotionProfiles.RPtoShip2R, Direction.kRevFlipped));
    //addSequentional(new PathFollower5010(RobotMap_Paths.finalShip2R));    
    //addSequential(new VisionReleaseHatch());

    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
