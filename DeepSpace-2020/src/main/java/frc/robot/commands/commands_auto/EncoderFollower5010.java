package frc.robot.commands.commands_auto;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.commands_auto.PathFollower5010.Direction;
import frc.robot.subsystems.DirectionSensor;
import frc.robot.util.Logger;
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
public class EncoderFollower5010 {

    boolean isRight; PathFollower5010.Direction isFwd;
    int encoder_offset, encoder_tick_count, last_segment_max, last_segment_count = 0;
    double wheel_circumference;

    double kp, ki, kd, kv, ka, ket, kht;
    double vint;
    double last_error, heading, last_heading_error;

    int segment;
    Trajectory trajectory;
    Trajectory.Segment next_segment;
    Logger log;
    Encoder encoder;
    DirectionSensor gyro;

    /** Constructor
    * @param traj a previously generated trajectory
    * @param isFwd                 Boolean, false = reverse, true = forward
    * @param isRight               Boolean, false = left, true = right
    */
    public EncoderFollower5010(Trajectory traj, boolean isRight, PathFollower5010.Direction isFwd) {
        this.trajectory = traj;
        this.isFwd = isFwd;
        this.isRight = isRight;
        last_segment_max = 10;
        log = new Logger(this.getClass().getSimpleName() + (isRight ? "_right_" : "_left_") + isFwd);
    }

    // Private so no one can use it, incorrectly
    private EncoderFollower5010() { }

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
    public void configurePIDVA(double kp, double ki, double kd, double kv, double ka, double ket, double kht, double vint) {
        this.kp = kp; this.ki = ki; this.kd = kd;
        this.kv = kv; this.ka = ka; this.ket = ket; this.kht = kht ; this.vint= vint;
    }

    /**
     * Configure the Encoders being used in the follower.
     * @param initial_position      The initial 'offset' of your encoder. This should be set to the encoder value just
     *                              before you start to track
     * @param ticks_per_revolution  How many ticks per revolution the encoder has
     * @param wheel_diameter        The diameter of your wheels (or pulleys for track systems) in meters
     */
    public void configureEncoder(Encoder encoder, DirectionSensor gyro, int ticks_per_revolution, double wheel_diameter) {
        this.encoder = encoder;
        this.gyro = gyro;
        encoder_offset = encoder.get();
        encoder_tick_count = ticks_per_revolution;
        wheel_circumference = Math.PI * wheel_diameter;
    }

    /**
     * Reset the follower to start again. Encoders must be reconfigured.
     */
    public void reset() {
        last_error = 0; segment = 0;
        last_heading_error = 0; next_segment = null;
        encoder_offset = 0; last_segment_count = 0;
        log.clear();
    }

    /**
     * Calculate the desired output for the motors, based on the amount of ticks the encoder has gone through.
     * This does not account for heading of the robot. To account for heading, add some extra terms in your control
     * loop for realignment based on gyroscope input and the desired heading given by this object.
     * @param encoder_tick  The amount of ticks the encoder has currently measured.
     * @param gHeading      Current gyro heading
     * @return              The desired output for your motor controller
     */
    public double calculate() {
        int encoder_tick = encoder.get();
        double gyro_heading = gyro.angle();
        // Number of Revolutions * Wheel Circumference
        if (isFwd != Direction.kForward) { encoder_tick = -encoder_tick; }
        double distance_covered = ((double)(encoder_tick - encoder_offset) / encoder_tick_count)
                * wheel_circumference;
        if (segment < trajectory.length()) {
            next_segment = trajectory.get(segment);
        } else { last_segment_count++; }

        if (!isFinished()) {
            double error = next_segment.position - distance_covered;
            double calculated_value =
                    kp * error +                                    // Proportional
                    kd * ((error - last_error) / next_segment.dt) +          // Derivative
                    (kv * next_segment.velocity + ka * next_segment.acceleration +vint); //all hail equation 14  
            last_error = error;
            
            heading = next_segment.heading;
            double desired_heading = Pathfinder.r2d(heading);
            if (isFwd == Direction.kForward) {
                // This moves to the else when PF fixed
                desired_heading = -desired_heading;
            } else if (isFwd == Direction.kRevNormal) {
                gyro_heading = 180 - gyro_heading;
            } else if (isFwd == Direction.kRevFlipped) {
                gyro_heading = - gyro_heading;
            }
            double last_heading_error = DirectionSensor.boundHalfDegrees(desired_heading - gyro_heading);
            double turn = 0.8 * (-1.0 / 80.0) * last_heading_error;

            log.entry("Out Cal", Double.valueOf(calculated_value));

            if (isRight) {
                calculated_value -= turn;
            } else {
                calculated_value += turn;
            }

            // Advance the segment, else check the calc value for minimal movement
            // This should ensure the final turn completes
            if (segment < trajectory.length()) {
                segment++;
            } else {
                calculated_value = Math.max(Math.abs(calculated_value), RobotMap.moveMin) * Math.signum(calculated_value);
            }
            if (isFwd != Direction.kForward) {
                calculated_value = -calculated_value;
            }

            double aVel = encoder.getRate();
            log.entry("Timestamp", Long.valueOf(RobotController.getFPGATime()));
            log.entry("Out Trn", Double.valueOf(calculated_value));
            log.entry("Pos Act", Double.valueOf(next_segment.position));
            log.entry("Pos Err", Double.valueOf(error));
            log.entry("Pos Exp", Double.valueOf(distance_covered));
            log.entry("Vel Act", Double.valueOf(aVel));
            log.entry("Vel Err", Double.valueOf(next_segment.velocity - aVel));
            log.entry("Vel Exp", Double.valueOf(next_segment.velocity));
            log.entry("ang Act", Double.valueOf(gyro_heading));
            log.entry("ang Err", Double.valueOf(next_segment.heading - gyro_heading));
            log.entry("ang Exp", Double.valueOf(next_segment.heading));
            log.writeEntries();
            return calculated_value;
        } else return 0;
    }

    public void end() {
        log.closeFileWriter();
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
        return next_segment;
    }

    /**
     * @return whether we have finished tracking this trajectory or not.
     */
    public boolean isFinished() {

        SmartDashboard.putNumber("last_error", last_error);
        SmartDashboard.putNumber("last_heading_error", last_heading_error);
        SmartDashboard.putNumber("last_segment_count", last_segment_count);
        SmartDashboard.putNumber("segment", segment);
        SmartDashboard.putNumber("trajectory_length", trajectory.length());

        return ((segment >= trajectory.length()) && 
        (Math.abs(last_error) < ket) && 
        (Math.abs(last_heading_error) < kht) ||
        last_segment_max < last_segment_count);
    }

}
