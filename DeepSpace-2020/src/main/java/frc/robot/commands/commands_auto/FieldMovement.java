/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.commands_auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotMap_Paths.MotionProfiles;
import frc.robot.commands.commands_auto.PathFollower5010.Direction;

public class FieldMovement extends CommandGroup {
  /**
   * Add your docs here.
   */
  public FieldMovement() {
    // Basic Test paths
   // addSequential(new PathFollower5010(MotionProfiles.testPath, Direction.kRevNormal));
    addSequential(new PathFollower5010(MotionProfiles.testPath, Direction.kForward));
   // addSequential(new PathFollower5010(MotionProfiles.testPath, Direction.kRevFlipped));
    //addSequential(new PathForward(RobotMap_Paths.testL,RobotMap_Paths.testR));
    //addSequential(new PathReverse(RobotMap_Paths.testL,RobotMap_Paths.testR));
    
    // encoder testing
    //addSequential(new DriveForDistance(50));

    // Basic test paths

    // Actual routines
    // addSequential(new PathFollower5010(RobotMap_Paths.exit_level_two_left, RobotMap_Paths.exit_level_two_right, Direction.kForward));

    // // Add Commands here:
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
