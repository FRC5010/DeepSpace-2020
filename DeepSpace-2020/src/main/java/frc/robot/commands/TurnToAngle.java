/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Robot;
import frc.robot.subsystems.DirectionSensor;

public class TurnToAngle extends PIDCommand {

	private static double p = 0.6;
	private static double i = 0.00;
	private static double d = 0.2;
	private double tolerance = 0.25;
	private double setPoint;

	private static final double driveTrainWidth = 24.375;

	public TurnToAngle(double setPoint) {
		super("TurnToAngle", p, i, d);
		setSetpoint(setPoint);
		this.setPoint = setPoint;
		requires(RobotMap.driveTrain);
		requires(RobotMap.direction);
		getPIDController().setAbsoluteTolerance(tolerance);
		getPIDController().setOutputRange(-.5, .5);
		getPIDController().setInputRange(-360, 361);

	}

	@Override
	protected double returnPIDInput() {

		return DirectionSensor.boundHalfDegrees(RobotMap.direction.angle());
	}

	protected void execute() {
		SmartDashboard.putNumber("angle", RobotMap.direction.angle());

	}

	protected void initialize() {
		setSetpoint(setPoint);
		// RobotMap.direction.reset();
		getPIDController().setPID(SmartDashboard.getNumber("P", 0.01), SmartDashboard.getNumber("I", 0),
				SmartDashboard.getNumber("D", 0));
	}

	@Override
	protected void usePIDOutput(double output) {
    if(output < RobotMap.moveMin){
      output = RobotMap.moveMin;
    }
    RobotMap.driveTrain.drive(-output, output);
	}

	protected void end() {

		RobotMap.driveTrain.stop();

	}

	protected void interrupted() {
		end();
	}

	@Override
	protected boolean isFinished() {
		SmartDashboard.putNumber("Error", getPIDController().getError());
		return getPIDController().onTarget() ||
		Robot.oi.driveTrainForward.getValue()!=0 ||
		Robot.oi.driveTrainTurn.getValue() !=0;


	}

}
