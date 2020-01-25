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
import frc.robot.commands.WristMM;
import frc.robot.commands.LimeLightState.State;
import frc.robot.commands.commands_auto.PathFollower5010;
import frc.robot.commands.commands_auto.PathFollower5010.Direction;
import frc.robot.subsystems.Wrist;

public class MiddleShipRight extends CommandGroup {
  /**
   * Add your docs here.
   */
  public MiddleShipRight() {
    addSequential(new Preload());

    // Middle start to Ship right-front bay
    addSequential(new ShiftUp());
    addParallel(new WristMM(Wrist.Position.LOW));
    addSequential(new PathFollower5010(MotionProfiles.MStoShip1R, Direction.kForward));
    addSequential(new LimeLightState(State.AUTO));
    addSequential(new ShiftDown());
    addSequential(new VADriveUntilDistance(25));
    addSequential(new ShiftUp());
    addParallel(new LimeLightState(State.DRIVER));
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
