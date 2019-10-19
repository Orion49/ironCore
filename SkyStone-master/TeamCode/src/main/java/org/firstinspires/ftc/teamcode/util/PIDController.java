package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

public class PIDController {

    private double error, lastError, errorSum;
    private double setpoint, tolerance;
    private double minInput, maxInput, minOutput, maxOutput, maxSum;
    private PIDCoefficients pidCoefficients;
    private double lastTime;
    private boolean inputBounded, outputBounded;

    //----------------------------------------------------------------------------------------------
    // Constructors
    //----------------------------------------------------------------------------------------------

    public PIDController(PIDCoefficients pidCoefficients) {
        this.pidCoefficients = pidCoefficients;
        lastTime = Double.NaN;
        maxSum = Double.POSITIVE_INFINITY;
    }

    public PIDController(PIDCoefficients pidCoefficients, double minInput, double maxInput, double minOutput, double maxOutput, double maxSum) {
        this.pidCoefficients = pidCoefficients;
        inputBounded = true;
        outputBounded = true;
        this.minInput = minInput;
        this.maxInput = maxInput;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        this.maxSum = maxSum;
        lastTime = Double.NaN;
    }

    //----------------------------------------------------------------------------------------------
    // Getters and Setters
    //----------------------------------------------------------------------------------------------

    public PIDCoefficients getPidCoefficients() {
        return pidCoefficients;
    }

    /**
     * Sets the percentage error which is considered tolerable for use with the onTarget method
     * @param tolerance decimal error which is tolerable (input of 0.15 = 15 percent tolerance)
     */
    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    /**
     * Sets the maximum value of the sum of the error in the calculate method so that the integral
     * term does not windup and overwhelm the output given;
     * @param maxIntegral maximum value of the integral term in the calculate method
     */
    public void setMaxSum(double maxIntegral) {
        this.maxSum = maxIntegral;
    }

    public void setInputRange(double minInput, double maxInput) {
        this.minInput = minInput;
        this.maxInput = maxInput;
        setSetpoint(setpoint);
    }

    public void setOutputRange(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        setSetpoint(setpoint);
    }

    /**
     * sets the set point (target position) of the PIDController, clipping it to between minInput
     * and maxInput
     * @param setpoint the set point of the PIDController
     */
    public void setSetpoint(double setpoint) {
        if (maxInput > minInput) {
           this.setpoint = Range.clip(setpoint, minInput, maxInput);
        } else {
            this.setpoint = setpoint;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Methods
    //----------------------------------------------------------------------------------------------

    /**
     * Calculates error between setpoint and input of the system, wrapping error if input bounds
     * have been set
     * @param input current position of the system
     */
    private void getError(double input) {
        error = setpoint - input;

        //wrapping error if input is bounded
        if(inputBounded) {
            double inputRange = maxInput - minInput;
            while (Math.abs(error) > inputRange / 2)
                error -= Math.signum(error) * inputRange;
        }
    }

    /**
     * Runs a single iteration of the feedback update with the error calculated by the getError()
     * method, using a trapezoidal sum to approximate the integral of error with respect to time
     * and a difference quotient to approximate the derivative of error with respect to time
     * @param input current position of the system
     * @return the calculated correction
     */
    public double calculate(double input) {
        getError(input);

        double currentTime = System.nanoTime();
        if(Double.isNaN(lastTime)) {
            //special case for handling first iteration
            lastError = error;
            lastTime = currentTime;
            return 0.0;
        }
        //calculating change in time
        double dt = (currentTime - lastTime) / 1E9;
        lastTime = currentTime;

        //approximating integral using a trapezoidal sum
        errorSum += 0.5 * (error + lastError) * dt;
        //capping errorSum to prevent integral windup
        errorSum = Range.clip(errorSum, -maxSum, maxSum);
        //approximating derivative using a difference quotient
        double derivative = (error - lastError) / dt;
        lastError = error;

        //calculating output (where e(t) represents the error as a function of time) with kP, kI
        //and kD representing the proportional, integral, and derivative coefficients respectively:
        //output(t) = e(t) * kP + ∫(e(t))dt * kI + de(t)/dt * kD
        double output = error * pidCoefficients.p + errorSum * pidCoefficients.i + derivative * pidCoefficients.d;
        return outputBounded ? Range.clip(output, minOutput, maxOutput) : output;
    }

    /**
     * Returns true if the error is within the tolerable zone of the input range determined by
     * setTolerance. Assumes that the input bounds were set using the setInputBounds method.
     * @return true if error is tolerable
     */
    public boolean onTarget() {
        if(inputBounded)
            return (Math.abs(error) < tolerance * (maxInput - minInput));
        else
            return false;
    }

    public void reset() {
        errorSum = 0.0;
        lastError = 0.0;
        lastTime = Double.NaN;
    }

    public String telemetryData() {
        return String.format("setpoint: %.2f, error: %.4f, errorSum: %.4f", setpoint, error, errorSum);
    }
}
