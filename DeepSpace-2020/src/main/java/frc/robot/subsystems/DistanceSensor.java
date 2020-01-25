package frc.robot.subsystems;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DistanceSensor extends Subsystem {
	
	public static double rightDPP = 0.5 * Math.PI / 120;
	// Uses 120 pulses per rotation from k1X 	
	
	public DistanceSensor() {
		RobotMap.rightEncoder.reset();
		RobotMap.leftEncoder.reset();
		
		RobotMap.rightEncoder.setDistancePerPulse(rightDPP);
		RobotMap.leftEncoder.setDistancePerPulse(rightDPP);
		
		RobotMap.rightEncoder.setReverseDirection(true);		
	}

	public DistanceSensor(String name) {
		super(name);
	}
	public double getDistance() {
		SmartDashboard.putNumber("right encoder getRaw", RobotMap.rightEncoder.get());
		
		SmartDashboard.putNumber("left encoder getRaw", RobotMap.leftEncoder.get());
		SmartDashboard.putNumber("left encoder get Distance",RobotMap.leftEncoder.getDistance());
		SmartDashboard.putNumber("right encoder get distance", RobotMap.rightEncoder.getDistance());
		return RobotMap.rightEncoder.getDistance();
	}
	
	public int getLeftRaw() {
		SmartDashboard.putNumber("left encoder getRaw", RobotMap.leftEncoder.get());
		return RobotMap.leftEncoder.getRaw();
	}
	
	public int getRightRaw() {
		SmartDashboard.putNumber("right encoder getRaw", RobotMap.rightEncoder.get());
		return RobotMap.rightEncoder.getRaw();		
	}

	public double getRightRate() {
		SmartDashboard.putNumber("right encoder getRate", RobotMap.rightEncoder.getRate());
		return RobotMap.rightEncoder.getRate();
	}

	public double getLeftRate() {
		SmartDashboard.putNumber("left encoder getRate", RobotMap.leftEncoder.getRate());
		return RobotMap.leftEncoder.getRate();
	}

	public void reset() {
		RobotMap.rightEncoder.reset();
		RobotMap.leftEncoder.reset();
	}
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
	}

}
