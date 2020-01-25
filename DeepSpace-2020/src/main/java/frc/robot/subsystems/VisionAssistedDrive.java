/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class VisionAssistedDrive {

  class PIDValues {
    public double kp, ki, kd, min;
    
    public PIDValues () {
      this.kp = this.ki = this.kd = this.min = 0;
    }

    public PIDValues (double kp, double ki, double kd) {
      this.kp = kp;
      this.ki = ki;
      this.kd = kd;
    }

    public PIDValues (double kp, double ki, double kd, double min) {
      this.kp = kp;
      this.ki = ki;
      this.kd = kd;
      this.min = min;
    }
  }

  class GearPID {
    public PIDValues steer, move, turnInPlace;
    
    public GearPID(PIDValues steer, PIDValues move, PIDValues turnInPlace) {
      this.steer = steer;
      this.move = move;
      this.turnInPlace = turnInPlace;
    }
  }

  public GearPID lowGear = new GearPID(
    new PIDValues(0.005, 0, 0.05, 0.02), 
    new PIDValues(0.01, 0, 0.01, 0.1),
    new PIDValues(0.01, 0, 0, 0.1)
    );
  public GearPID highGear = new GearPID(
    new PIDValues(0.015, 0, 0, 0.08),
    new PIDValues(0.06, 0, 0.2, 0.08),
    new PIDValues(0.08, 0, 0.5, 0.1)
    );

  public double getSteerKp() {
    return getSteerKp(false);
  }

  public double getSteerKp(boolean turnInPlace) {
    if (turnInPlace) {
      lowGear.turnInPlace.kp = SmartDashboard.getNumber("lowGear.turnInPlace.kp", lowGear.turnInPlace.kp);
      highGear.turnInPlace.kp = SmartDashboard.getNumber("highGear.turnInPlace.kp", highGear.turnInPlace.kp);
      return Shifter.isLowGear ? lowGear.turnInPlace.kp : highGear.turnInPlace.kp;
    } else {
      lowGear.steer.kp = SmartDashboard.getNumber("lowGear.steer.kp", lowGear.steer.kp);
      highGear.steer.kp = SmartDashboard.getNumber("highGear.steer.kp", highGear.steer.kp);
      return Shifter.isLowGear ? lowGear.steer.kp : highGear.steer.kp;
    }
  }

  public double getSteerKi() {
    return getSteerKi(false);
  }

  public double getSteerKi(boolean turnInPlace) {
    if (turnInPlace) {
      lowGear.turnInPlace.ki = SmartDashboard.getNumber("lowGear.turnInPlace.ki", lowGear.turnInPlace.ki);
      highGear.turnInPlace.ki = SmartDashboard.getNumber("highGear.turnInPlace.ki", highGear.turnInPlace.ki);
      return Shifter.isLowGear ? lowGear.turnInPlace.ki : highGear.turnInPlace.ki;
    } else {
      lowGear.steer.ki = SmartDashboard.getNumber("lowGear.steer.ki", lowGear.steer.ki);
      highGear.steer.ki = SmartDashboard.getNumber("highGear.steer.ki", highGear.steer.ki);
      return Shifter.isLowGear ? lowGear.steer.ki : highGear.steer.ki;
    }
  }

  public double getSteerKd() {
    return getSteerKd(false);
  }

  public double getSteerKd(boolean turnInPlace) {
    if (turnInPlace) {
      lowGear.turnInPlace.kd = SmartDashboard.getNumber("lowGear.turnInPlace.kd", lowGear.turnInPlace.kd);
      highGear.turnInPlace.kd = SmartDashboard.getNumber("highGear.turnInPlace.kd", highGear.turnInPlace.kd);
      return Shifter.isLowGear ? lowGear.turnInPlace.kd : highGear.turnInPlace.kd;
    } else {
      lowGear.steer.kd = SmartDashboard.getNumber("lowGear.steer.kd", lowGear.steer.kd);
      highGear.steer.kd = SmartDashboard.getNumber("highGear.steer.kd", highGear.steer.kd);
      return Shifter.isLowGear ? lowGear.steer.kd : highGear.steer.kd;
    }
  }

  public double getSteerMin() {
    return getSteerKd(false);
  }

  public double getSteerMin(boolean turnInPlace) {
    if (turnInPlace) {
      lowGear.turnInPlace.min = SmartDashboard.getNumber("lowGear.turnInPlace.min", lowGear.turnInPlace.min);
      highGear.turnInPlace.min = SmartDashboard.getNumber("highGear.turnInPlace.min", highGear.turnInPlace.min);
      return Shifter.isLowGear ? lowGear.turnInPlace.min : highGear.turnInPlace.min;
    } else {
      lowGear.steer.min = SmartDashboard.getNumber("lowGear.steer.min", lowGear.steer.min);
      highGear.steer.min = SmartDashboard.getNumber("highGear.steer.min", highGear.steer.min);
      return Shifter.isLowGear ? lowGear.steer.min : highGear.steer.min;
    }
  }

  public double getMoveKp() {
    lowGear.move.kp = SmartDashboard.getNumber("lowGear.move.kp", lowGear.move.kp);
    highGear.move.kp = SmartDashboard.getNumber("highGear.move.kp", highGear.move.kp);
    return Shifter.isLowGear ? lowGear.move.kp : highGear.move.kp;
  }

  public double getMoveKi() {
    lowGear.move.ki = SmartDashboard.getNumber("lowGear.move.ki", lowGear.move.ki);
    highGear.steer.ki = SmartDashboard.getNumber("highGear.steer.ki", highGear.steer.ki);
    return Shifter.isLowGear ? lowGear.move.ki : highGear.move.ki;
  }

  public double getMoveKd() {
    lowGear.move.kd = SmartDashboard.getNumber("lowGear.move.kd", lowGear.move.kd);
    highGear.steer.kd = SmartDashboard.getNumber("highGear.steer.kd", highGear.steer.kd);
    return Shifter.isLowGear ? lowGear.move.kd : highGear.move.kd;
  }

  public double getMoveMin() {
    lowGear.move.min = SmartDashboard.getNumber("lowGear.move.min", lowGear.move.min);
    highGear.move.min = SmartDashboard.getNumber("highGear.move.min", highGear.move.min);
    return Shifter.isLowGear ? lowGear.move.min : highGear.move.min;
  }

  public void printPIDValues() {
    SmartDashboard.putNumber("lowGear.steer.kp", lowGear.steer.kp);
    SmartDashboard.putNumber("lowGear.steer.ki", lowGear.steer.ki);
    SmartDashboard.putNumber("lowGear.steer.kd", lowGear.steer.kd);
    SmartDashboard.putNumber("lowGear.steer.min", lowGear.steer.min);
    
    SmartDashboard.putNumber("lowGear.turnInPlace.kp", lowGear.turnInPlace.kp);
    SmartDashboard.putNumber("lowGear.turnInPlace.ki", lowGear.turnInPlace.ki);
    SmartDashboard.putNumber("lowGear.turnInPlace.kd", lowGear.turnInPlace.kd);
    SmartDashboard.putNumber("lowGear.turnInPlace.min", lowGear.turnInPlace.min);

    SmartDashboard.putNumber("lowGear.move.kp", lowGear.move.kp);
    SmartDashboard.putNumber("lowGear.move.ki", lowGear.move.ki);
    SmartDashboard.putNumber("lowGear.move.kd", lowGear.move.kd);
    SmartDashboard.putNumber("lowGear.move.min", lowGear.move.min);

    SmartDashboard.putNumber("highGear.steer.kp", highGear.steer.kp);
    SmartDashboard.putNumber("highGear.steer.ki", highGear.steer.ki);
    SmartDashboard.putNumber("highGear.steer.kd", highGear.steer.kd);
    SmartDashboard.putNumber("highGear.steer.min", highGear.steer.min);

    SmartDashboard.putNumber("highGear.turnInPlace.kp", highGear.turnInPlace.kp);
    SmartDashboard.putNumber("highGear.turnInPlace.ki", highGear.turnInPlace.ki);
    SmartDashboard.putNumber("highGear.turnInPlace.kd", highGear.turnInPlace.kd);
    SmartDashboard.putNumber("highGear.turnInPlace.min", highGear.turnInPlace.min);

    SmartDashboard.putNumber("highGear.move.kp", highGear.move.kp);
    SmartDashboard.putNumber("highGear.move.ki", highGear.move.ki);
    SmartDashboard.putNumber("highGear.move.kd", highGear.move.kd);
    SmartDashboard.putNumber("highGear.move.min", highGear.move.min);
  }


  // arc towards target
  static double minRotationDistance = 40;
  public double arcTowardsTarget() {
    Pose currentPose = Pose.getCurrentPose();
    if (currentPose.limeLight.tValid) {
      double distance = currentPose.limeLight.tDistance;
      System.out.println("-------");
      double rotationAngle = currentPose.limeLight.tX;
      System.out.println("Angle: " + rotationAngle);
      double approachAngle = currentPose.limeLight.tDistance / 3;
      System.out.println("Approach Angle: " + approachAngle);
      if (approachAngle > 20) {
        approachAngle = 20;
      }
      if (currentPose.limeLight.leftRightRatio > 0) {
        approachAngle = -approachAngle;
      }
      System.out.println("Desired Angle: " + approachAngle);
      
      double error = approachAngle - rotationAngle;
      double steerAmt = 0.01 * error; //(Shifter.isLowGear ? lowGear.steerKp : highGear.steerKp)
      System.out.println("SteerAmt: " + steerAmt);

      // double moveMin = Shifter.isLowGear ? lowGear.moveMin : highGear.moveMin;
      // steerAmt = Math.signum(steerAmt) * Math.max(Math.abs(steerAmt), moveMin);
      SmartDashboard.putNumber(this.getClass().getSimpleName() + " Steer", steerAmt);

      return steerAmt;
    }
    return 0.0;
  }
}
