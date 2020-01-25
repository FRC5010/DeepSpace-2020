/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.dynasty;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickAxis {
    private final Joystick joystick;
    private final int axis;

    private double deadzone = 0.15;
    private double lower = -1;
    private double upper = 1;
    private double range = 1;
    private int factor = 3;
    private boolean inversed = false;

    /** Define joystick & axis with defaults */
    public JoystickAxis(Joystick pjoystick, int paxis) {
      joystick = pjoystick;
      axis = paxis;
    }
    
    /** Define joystick, axis & inversion*/
    public JoystickAxis(Joystick pjoystick, int paxis, boolean inversed) {
        joystick = pjoystick;
        axis = paxis;
        this.inversed = inversed;
    }

    /** Define joystick, axis & range */
    public JoystickAxis(Joystick pjoystick, int paxis, double range) {
        joystick = pjoystick;
        axis = paxis;
        setRange(range);
    }

    /** Define joystick, axis, inversion & range */
    public JoystickAxis(Joystick pjoystick, int paxis, boolean inversed, double range) {
        joystick = pjoystick;
        axis = paxis;
        this.inversed = inversed;
        setRange(range);
    }
   
    /** The deadzone is the input range of the joystick to ignore */
    public double getDeadzone() { return deadzone; }
    /** The lowest output value (usually negative).
     * Can not be less than -(range)
    */
    public double getLowerLimit() { return lower; }
    /** The highest output value. 
     * Can not be greater than range
    */
    public double getUpperLimit() { return upper; }
    /** The absolute scale of the output */
    public double getRange() { return range; }
    /** Change negative input to positive output and vice versa */
    public boolean isInverted() { return inversed; }
    /** The behavioral power-curve, used by Math.power */
    public int getFactor() { return factor; }

    /** The deadzone is the input range of the joystick to ignore */
    public void setDeadzone(double deadzone) { this.deadzone = deadzone; }
    /** The lowest output value (usually negative).
     * Can not be less than -(range)
    */
    public void setLowerLimit(double min) { 
        this.lower = Math.max(min, -range); 
    }
    /** The highest output value.
     * Can not be greater than range
    */
    public void setUpperLimit(double max) { 
        this.upper = Math.min(max, range); 
    }
    /** The absolute scale of the output 
     * WARNING: This sets upper and lower limits) */
    public void setRange(double range) { 
        this.range = range; 
        upper = range;
        lower = -range;
    }
    /** Change negative input to positive output and vice versa */
    public void setInverted(boolean inverted) { inversed = inverted; }
    /** The behavioral power-curve, used by Math.power */
    public void setFactor(int factor) {
        if (factor < 1) { factor = 1; }
        this.factor = factor; 
    }

    /** Returns the output calculated for this joystick */
    public double getValue() {
      return scaleInputs(joystick.getRawAxis(axis));
    }

    private double scaleInputs(double input) {
        // Invert first, so that calculations are against correct range
        input = input * (inversed ? -1 : 1);
        double output = input;
        if(deadzone > 0) {
            if (Math.abs(input) < deadzone) {
                output = 0;
            } else if (input > 0) {
                output = (input - deadzone) / (1 - deadzone);
                if (factor > 1) { // Sanity check
                    output = Math.pow(output, factor);
                }
            } else if (input < 0) {
                output = (input + deadzone) / (1 - deadzone);
                if (factor > 1) { // Sanity check
                    output = Math.pow(output, factor);
                    // Even numbered factors could result in swapping sign
                    if (Math.signum(output) != -1) {
                        output *= -1;
                    }
                }
            }
        }
        // Multiply range here once power curve is computed
        return clamp(range * output);
    }

    // "clamps" a value between the range min and max
    private double clamp(double value) {
        if (lower <= upper) {
            value = Math.min(Math.max(lower, value), upper);
        }
        return value;
    }
  }
