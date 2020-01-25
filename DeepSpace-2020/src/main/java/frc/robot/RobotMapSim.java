/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.BeakIntake;
import frc.robot.subsystems.DirectionSensor;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.VisionAssistedDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.util.Constants;
/**
 * Add your docs here.
 */
public class RobotMapSim {

    public static void initSim() {
        RobotMap_Paths.deployPath = Filesystem.getOperatingDirectory().getAbsolutePath() + "/src/main/deploy/paths/";

        RobotMap_Paths.init();
        initMotors();
        RobotMap.rightEncoder = new Encoder(0, 1);
        RobotMap.leftEncoder = new Encoder(2,3);
        RobotMap.encoderPPR=480;
        
        RobotMap.distance = new DistanceSensor();
        RobotMap.direction = new DirectionSensor(null);
        RobotMap.direction.reset();
    
        RobotMap.elevator = new Elevator(RobotMap.elevatorMotor, RobotMap.elevatorMotor2);
        RobotMap.wrist = new Wrist();
        RobotMap.shifter = new Shifter();
        RobotMap.driveTrain = new DriveTrain();
        RobotMap.ballIntake = new BallIntake();
        RobotMap.beakIntake = new BeakIntake();
        RobotMap.vision = new Vision();
        RobotMap.visionDrive = new VisionAssistedDrive();
      }

      public static void initMotors() {
        RobotMap.rightMotor1 = new WPI_TalonSRX(4);
        RobotMap.rightMotor2 = new WPI_TalonSRX(6);
    
        RobotMap.leftMotor1 = new WPI_TalonSRX(2);
        RobotMap.leftMotor2 = new WPI_TalonSRX(5);
        
        RobotMap.elevatorMotor = new WPI_TalonSRX(3);
        RobotMap.elevatorMotor2 = new WPI_TalonSRX(1);
    
        RobotMap.intakeMotor = new WPI_VictorSPX(0);
        
        RobotMap.rightMotor1.setInverted(true);
        RobotMap.rightMotor2.setInverted(true);
    
        RobotMap.rightMotor2.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 4);
        RobotMap.leftMotor2.set(com.ctre.phoenix.motorcontrol.ControlMode.Follower, 2);
    
        RobotMap.wristMotor = new WPI_TalonSRX(7);
      }
}



