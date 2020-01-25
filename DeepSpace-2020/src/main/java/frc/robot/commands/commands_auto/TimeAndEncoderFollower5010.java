package frc.robot.commands.commands_auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.commands_auto.TimedPathFollower5010.Direction;
import frc.robot.subsystems.DirectionSensor;
import frc.robot.subsystems.Vision;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

/**
 * The EncoderFollower is an object designed to follow a trajectory based on encoder input. This class can be used
 * for Tank or Swerve drive implementations.
 *
 * @author Jaci
 * 
 * Modified to repeat the last segment until error is in an acceptable range.
 */
public class TimeAndEncoderFollower5010 {

    boolean isRight; TimedPathFollower5010.Direction isFwd;
    int encoder_offset, encoder_tick_count, last_segment_max, last_segment_count = 0;
    double wheel_circumference;

    double kp, ki, kd, kv, ka, ket, kht;

    double last_error, heading, last_heading_error, last_elapsed_time, segment_delta;

    int segment;
    Trajectory trajectory;
    Trajectory.Segment next_segment, prev_segment, current_segment;

    /** Constructor
    * @param traj a previously generated trajectory
    * @param isFwd                 Boolean, false = reverse, true = forward
    * @param isRight               Boolean, false = left, true = right
    */
    public TimeAndEncoderFollower5010(Trajectory traj, boolean isRight, TimedPathFollower5010.Direction isFwd) {
        this.trajectory = traj;
        this.isFwd = isFwd;
        this.isRight = isRight;
        last_segment_max = 10;
        last_elapsed_time = 0;
        next_segment = null;
        prev_segment = null;
        next_segment = trajectory.get(0);
        segment_delta = next_segment.dt;
    }

    // Private so no one can use it, incorrectly
    private TimeAndEncoderFollower5010() { }

    /** The number of cycles limit the last segment to */
    public void setLastSegmentMax(int max) {
        last_segment_max = max;
    }

    /**
     * Set a new trajectory to follow, and reset the cumulative errors and segment counts
     * @param traj a previously generated trajectory
     */
    public void setTrajectory(Trajectory traj) {
        this.trajectory = traj;
        reset();
    }

    /**
     * Configure the PID/VA Variables for the Follower
     * @param kp The proportional term. This is usually quite high (0.8 - 1.0 are common values)
     * @param ki The integral term. Currently unused.
     * @param kd The derivative term. Adjust this if you are unhappy with the tracking of the follower. 0.0 is the default
     * @param kv The velocity ratio. This should be 1 over your maximum velocity @ 100% throttle.
     *           This converts m/s given by the algorithm to a scale of -1..1 to be used by your
     *           motor controllers
     * @param ka The acceleration term. Adjust this if you want to reach higher or lower speeds faster. 0.0 is the default
     * @param ket The encoder error tolerance to achieve before isFinished will return true
     * @param kht The heading error tolerance to achieve before isFinished will return true
     */
    public void configurePIDVA(double kp, double ki, double kd, double kv, double ka, double ket, double kht) {
        this.kp = kp; this.ki = ki; this.kd = kd;
        this.kv = kv; this.ka = ka; this.ket = ket; this.kht = kht;
    }

    /**
     * Configure the Encoders being used in the follower.
     * @param initial_position      The initial 'offset' of your encoder. This should be set to the encoder value just
     *                              before you start to track
     * @param ticks_per_revolution  How many ticks per revolution the encoder has
     * @param wheel_diameter        The diameter of your wheels (or pulleys for track systems) in meters
     */
    public void configureEncoder(int initial_position, int ticks_per_revolution, double wheel_diameter) {
        encoder_offset = initial_position;
        encoder_tick_count = ticks_per_revolution;
        wheel_circumference = Math.PI * wheel_diameter;
    }

    /**
     * Reset the follower to start again. Encoders must be reconfigured.
     */
    public void reset() {
        last_error = 0; segment = 0;
        last_heading_error = 0;
        last_elapsed_time = 0;
        encoder_offset = 0; last_segment_count = 0;
        prev_segment = null;
        next_segment = trajectory.get(0);
    }

    /**
     * Calculate the desired output for the motors, based on the amount of ticks the encoder has gone through.
     * This does not account for heading of the robot. To account for heading, add some extra terms in your control
     * loop for realignment based on gyroscope input and the desired heading given by this object.
     * @param encoder_tick  The amount of ticks the encoder has currently measured.
     * @param gHeading      Current gyro heading
     * @return              The desired output for your motor controller
     */
    public double calculate(int encoder_tick, double gyro_heading, double elapsed_time) {
        current_segment = next_segment;
        // Number of Revolutions * Wheel Circumference
        if (isFwd != Direction.kForward) { encoder_tick = -encoder_tick; }
        double distance_covered = ((double)(encoder_tick - encoder_offset) / encoder_tick_count)
                * wheel_circumference;
        if (elapsed_time > 0) {
            double derivative_time_delta = elapsed_time - last_elapsed_time;
            double segment_time = elapsed_time / segment_delta;
            int segment_prev = ((int)Math.floor(segment_time));
            int segment_next = ((int)Math.ceil(segment_time));
            System.out.println("Previous Segment: " + segment_prev);
            if (segment_prev < trajectory.length()) {
                segment = segment_prev;
                prev_segment = trajectory.get(segment_prev);
                if (segment_next < trajectory.length()) {
                    next_segment = trajectory.get(segment_next);
                    double t1 = segment_prev * segment_delta;
                    double t2 = segment_next * segment_delta;
                        double position, velocity, acceleration, heading;
                    position = Vision.Values.linearMidInterpolation(t1, prev_segment.position, 
                        t2, next_segment.position, elapsed_time);
                    velocity = Vision.Values.linearMidInterpolation(t1, prev_segment.velocity, 
                        t2, next_segment.velocity, elapsed_time);
                    acceleration = Vision.Values.linearMidInterpolation(t1, prev_segment.acceleration, 
                        t2, next_segment.acceleration, elapsed_time);
                    heading = Vision.Values.linearMidInterpolation(t1, prev_segment.heading, 
                        t2, next_segment.heading, elapsed_time);
                    current_segment = new Trajectory.Segment(derivative_time_delta, 0, 0, position, velocity, acceleration, 0, heading);
                } else {
                    current_segment = prev_segment;
                }
            } else { 
                segment = trajectory.length() - 1;
                current_segment = trajectory.get(segment);
                last_segment_count++; 
            }
            current_segment.dt = derivative_time_delta;
        } else {
            segment = 0;
            current_segment = next_segment;
        }

        if (!isFinished()) {
            double error = current_segment.position - distance_covered;
            double calculated_value =
                    kp * error +                                    // Proportional
                    kd * ((error - last_error) / current_segment.dt) +          // Derivative
                    (kv * current_segment.velocity + ka * current_segment.acceleration);    // V and A Terms
            last_error = error;
            
            heading = current_segment.heading;
            double desired_heading = Pathfinder.r2d(heading);
            if (isFwd == Direction.kForward) {
                // This moves to the else when PF fixed
                desired_heading = -desired_heading;
            } else if (isFwd == Direction.kRevNormal) {
                gyro_heading = 180 - gyro_heading;
            } else if (isFwd == Direction.kRevFlipped) {
                gyro_heading = -gyro_heading;
            }
            double heading_error = DirectionSensor.boundHalfDegrees(desired_heading - gyro_heading);
            double turn = RobotMap.visionDrive.getSteerKp() * heading_error + 
                (RobotMap.visionDrive.getSteerKd() * ((heading_error - last_heading_error)/current_segment.dt));
            last_heading_error = heading_error;

            if (isRight) {
                calculated_value -= turn;
            } else {
                calculated_value += turn;
            }

            // Advance the segment, else check the calc value for minimal movement
            // This should ensure the final turn completes
            calculated_value = Math.max(Math.abs(calculated_value), RobotMap.moveMin) * Math.signum(calculated_value);
            if (isFwd != Direction.kForward) {
                calculated_value = -calculated_value;
            }

            return calculated_value;
        } else return 0;
    }

    /**
     * @return the desired heading of the current point in the trajectory
     */
    public double getHeading() {
        return heading;
    }

    /**
     * @return the current segment being operated on
     */
    public Trajectory.Segment getSegment() {
        return current_segment;
    }

    /**
     * @return whether we have finished tracking this trajectory or not.
     */
    public boolean isFinished() {

        SmartDashboard.putNumber(this.getClass().getSimpleName() + " last_error", last_error);
        SmartDashboard.putNumber(this.getClass().getSimpleName() + " last_heading_error", last_heading_error);
        SmartDashboard.putNumber(this.getClass().getSimpleName() + " last_segment_count", last_segment_count);
        SmartDashboard.putNumber(this.getClass().getSimpleName() + " segment", segment);
        SmartDashboard.putNumber(this.getClass().getSimpleName() + " trajectory_length", trajectory.length());

        return ((segment >= trajectory.length()) && 
        (Math.abs(last_error) < ket) && 
        (Math.abs(last_heading_error) < kht) ||
        last_segment_max < last_segment_count);
    }

}
