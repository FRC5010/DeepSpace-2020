/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.RaiseElevator;
import frc.robot.util.Constants;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  private final double feedForward = 0.08;
  public static double ZERO = 1;
  public static double HATCH_LOW = 1;
  public static double HATCH_MIDDLE = 7500;
  public static double HATCH_HIGH = 14857;
  public static double CARGO_LOW = 1;
  public static double CARGO_MIDDLE = 8000;
  public static double CARGO_HIGH = 14000;
  public static double CARGO_SHIP = 7500;
  public static double MAX_FWD_OUT = 1;
  public static double MAX_REV_OUT = -0.5;


  public static enum Position {
    LOW, MIDDLE, HIGH, SHIP
  }

  public boolean isCargoGamePiece = false;
  private long lastPosition = 0;
  private int numTimesAtLastPosition = 0;
  private WPI_TalonSRX elevMotor;
  private Faults faults = new Faults();

  public Elevator(WPI_TalonSRX elevatorMotor, WPI_TalonSRX elevatorMotor2) {
    elevMotor = elevatorMotor;
    elevatorMotor.configFactoryDefault();
    elevatorMotor2.configFactoryDefault();
    elevatorMotor2.follow(elevatorMotor);
    elevatorMotor.setNeutralMode(NeutralMode.Brake);
    elevatorMotor2.setNeutralMode(NeutralMode.Brake);

    elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);

    elevatorMotor.setSensorPhase(!RobotMap.isComp);
    elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
    elevatorMotor.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
    elevatorMotor2.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
    // configing outputs
    elevatorMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    elevatorMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    elevatorMotor.configPeakOutputForward(MAX_FWD_OUT, Constants.kTimeoutMs);
    elevatorMotor.configPeakOutputReverse(MAX_REV_OUT, Constants.kTimeoutMs);

    SmartDashboard.putNumber("Elevator P", Constants.kGains.kP);
    SmartDashboard.putNumber("Elevator I", Constants.kGains.kI);
    SmartDashboard.putNumber("Elevator D", Constants.kGains.kD);
    SmartDashboard.putNumber("Elevator F", Constants.kGains.kF);
    elevatorMotor.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    elevatorMotor.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    elevatorMotor.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    elevatorMotor.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
    elevatorMotor.configAllowableClosedloopError(Constants.kSlotIdx, 10, Constants.kTimeoutMs);
    elevatorMotor.config_IntegralZone(Constants.kSlotIdx, 100, Constants.kTimeoutMs);
    elevatorMotor.configClosedLoopPeakOutput(Constants.kSlotIdx, MAX_FWD_OUT, Constants.kTimeoutMs);

    elevatorMotor.config_kF(Constants.kSlotDown, Constants.kGains.kF, Constants.kTimeoutMs);
    elevatorMotor.config_kP(Constants.kSlotDown, Constants.kGains.kP, Constants.kTimeoutMs);
    elevatorMotor.config_kI(Constants.kSlotDown, Constants.kGains.kI, Constants.kTimeoutMs);
    elevatorMotor.config_kD(Constants.kSlotDown, Constants.kGains.kD, Constants.kTimeoutMs);
    elevatorMotor.configAllowableClosedloopError(Constants.kSlotDown, 10, Constants.kTimeoutMs);
    elevatorMotor.config_IntegralZone(Constants.kSlotDown, 100, Constants.kTimeoutMs);
    elevatorMotor.configClosedLoopPeakOutput(Constants.kSlotDown, MAX_REV_OUT, Constants.kTimeoutMs);

    elevatorMotor.configClosedLoopPeriod(0, 1, Constants.kTimeoutMs);
    elevatorMotor.configClosedLoopPeriod(1, 1, Constants.kTimeoutMs);

    // cruise velocity
    elevatorMotor.configMotionCruiseVelocity(3600, Constants.kTimeoutMs);
    elevatorMotor.configMotionAcceleration(2500, Constants.kTimeoutMs);

    // zeroing sensor
    elevatorMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    SmartDashboard.putNumber("Elevator Feed Forward", feedForward);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new RaiseElevator());
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void reset(){
    elevMotor.setSelectedSensorPosition(0);
  }

  /** Don't drive the motor past a point where it can't move for too long */
  public boolean isSomethingStuck(double power) {
    long curPose = getCurrentPosition();
    if (RobotMap.checkMotorSafety && 0.25 < power) {
      if (lastPosition / 10 == curPose / 10) {
        ++numTimesAtLastPosition;
      } else {
        numTimesAtLastPosition = 0;
      }
      lastPosition = curPose;
      return 3 <= numTimesAtLastPosition;
    } else {
      numTimesAtLastPosition = 0;
      return false;
    }
  }

  public void raiseElevator(double power) {
    if (isSomethingStuck(power)) {
      power = 0;
    }
    double feedForward = SmartDashboard.getNumber("Elevator Feed Forward", this.feedForward);
    if (100 > getCurrentPosition()) {
      feedForward = 0;
    }
    if (Pose.getCurrentPose().elevatorAmps > 65) {
      power = Math.min(power, elevMotor.getMotorOutputPercent() - 0.1);
    }
    elevMotor.set(ControlMode.PercentOutput, power, DemandType.ArbitraryFeedForward, feedForward);
  }

  public void moveToPosition(double setPoint) {
    // double kP = SmartDashboard.getNumber("Elevator P", 4);
    // double kI = SmartDashboard.getNumber("Elevator I", 0);
    // double kD = SmartDashboard.getNumber("Elevator D", 4);
    // double kF = SmartDashboard.getNumber("Elevator F", 0.2);
    // elevMotor.config_kF(Constants.kSlotIdx,kF, Constants.kTimeoutMs);
    // elevMotor.config_kP(Constants.kSlotIdx,kP, Constants.kTimeoutMs);
    // elevMotor.config_kI(Constants.kSlotIdx,kI, Constants.kTimeoutMs);
    // elevMotor.config_kD(Constants.kSlotIdx,kD, Constants.kTimeoutMs);
    double feedForward = SmartDashboard.getNumber("Elevator Feed Forward", this.feedForward);
    double curPose = getCurrentPosition();
//    if (curPose < setPoint) {
      elevMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    // } else {
    //   elevMotor.selectProfileSlot(Constants.kSlotDown, Constants.kPIDLoopIdx);
    // }
    elevMotor.set(ControlMode.MotionMagic, setPoint, DemandType.ArbitraryFeedForward, feedForward);
  }

  public long getCurrentPosition() {
    SmartDashboard.putNumber("Elevator position: ", elevMotor.getSelectedSensorPosition());
    return elevMotor.getSelectedSensorPosition();
  }

  public void returnToManualControl() {
    raiseElevator(0);
  }

  public void tuneElevator(double power) {
    elevMotor.set(ControlMode.PercentOutput, power);
    elevMotor.getFaults(faults);
    System.out.println("Sensor Vel: " + elevMotor.getSelectedSensorVelocity());
    System.out.println("Sensor Pos: " + elevMotor.getSelectedSensorPosition());
    System.out.println("Elev Out %: " + elevMotor.getMotorOutputPercent());
    System.out.println("Out of Phs: " + faults.SensorOutOfPhase);
  }
}
