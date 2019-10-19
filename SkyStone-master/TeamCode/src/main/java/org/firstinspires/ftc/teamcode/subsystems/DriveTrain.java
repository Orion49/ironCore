package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.PIDController;

import java.util.HashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.util.Constants.*;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;

public class DriveTrain extends Subsystem {

    public DcMotor[] motors;
    private static final String[] motorNames = {"left_drive", "right_drive", "strafe_drive"};

    //heading related PIVs
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    //pid controllers
    private PIDController pidDrive, pidStrafe, pidRotate;

    //----------------------------------------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------------------------------------

    public DriveTrain(LinearOpMode opMode, boolean autonomous) {
        super(opMode, autonomous);

        //configure motors
        motors = new DcMotor[motorNames.length];
        for(int i = 0; i < motorNames.length; i++) {
            motors[i] = opMode.hardwareMap.get(DcMotor.class, motorNames[i]);
            motors[i].setDirection(i == 0 ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            motors[i].setMode(STOP_AND_RESET_ENCODER);
            motors[i].setMode(RUN_WITHOUT_ENCODER);
        }

        //initialize IMU
        if(autonomous) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            opMode.telemetry.addData("Status", "calibrating imu");
            opMode.telemetry.update();

            while (!opMode.isStopRequested() && !imu.isGyroCalibrated()) {
                opMode.sleep(50);
                opMode.idle();
            }

            opMode.telemetry.addData("Status", "calibrated imu");
            opMode.telemetry.update();
        }

        //initialize PIDControllers
        pidDrive = new PIDController(DRIVE_PID_COEFFICIENTS);
        pidStrafe = new PIDController(STRAFE_PID_COEFFICIENTS);
        pidRotate = new PIDController(ROTATE_PID_COEFFICIENTS);

        opMode.telemetry.addData("Status", "DriveTrain instantiated");
        opMode.telemetry.update();
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry
    //----------------------------------------------------------------------------------------------

    @Override
    public Map<String, Object> updateTelemetry() {
        Map<String, Object> telemetryData = new HashMap<>();

        telemetryData.put("heading", globalAngle);
        for(int i = 0; i < motors.length; i++) {
            telemetryData.put(motorNames[i] + " encoder counts", motors[i].getCurrentPosition());
            telemetryData.put(motorNames[i] + " power", motors[i].getPower());
        }
        telemetryData.put("pidDrive", pidDrive.telemetryData());
        telemetryData.put("pidStrafe", pidStrafe.telemetryData());
        telemetryData.put("pidRotate", pidRotate.telemetryData());

        return telemetryData;
    }

    //----------------------------------------------------------------------------------------------
    // Miscellaneous Helper Methods
    //----------------------------------------------------------------------------------------------

    public void setPower(double... powers) {
        for(int i = 0; i < motors.length; i++)
            motors[i].setPower(powers[i]);
    }

    private void setPower(double power) {
        for(DcMotor motor: motors)
            motor.setPower(power);
    }

    private void setTargetPosition(int... targetPositions) {
        for(int i = 0; i < motors.length; i++)
            motors[i].setTargetPosition(targetPositions[i]);
    }

    private void setMode(DcMotor.RunMode... modes) {
        for(int i = 0; i < motors.length; i++)
            motors[i].setMode(modes[i]);
    }

    //----------------------------------------------------------------------------------------------
    // Autonomous Helper Methods
    //----------------------------------------------------------------------------------------------

    private static int ticks(double inches) {
        return (int) Math.round((inches / WHEEL_CIRCUMFERENCE * TICKS_PER_REVOLUTION * GEAR_RATIO));
    }

    /**
     * @return returns + when rotating counter clockwise (left) and - when rotating clockwise (right)
     */
    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        while(deltaAngle < -180)
            deltaAngle += 360;
        while(deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    //----------------------------------------------------------------------------------------------
    // Autonomous Non-PID Methods
    //----------------------------------------------------------------------------------------------

    /**
     * drives a given number of inches
     * @param inches positive when driving forwards, negative when driving backwards
     */
    public void drive(double inches) {
        int ticks = ticks(inches);
        int leftTarget = motors[0].getCurrentPosition() + ticks;
        int rightTarget = motors[1].getCurrentPosition() + ticks;

        setTargetPosition(leftTarget, rightTarget);
        setMode(RUN_TO_POSITION, RUN_TO_POSITION);

        setPower(DRIVE_SPEED, DRIVE_SPEED);
        while(opMode.opModeIsActive() && motors[0].isBusy() && motors[1].isBusy()) {
            motors[0].setPower(DRIVE_SPEED);
            motors[1].setPower(DRIVE_SPEED);
        }
        setPower(0);
        motors[0].setMode(RUN_WITHOUT_ENCODER);
        motors[1].setMode(RUN_WITHOUT_ENCODER);
    }

    /**
     * strafes a given number of inches
     * @param inches positive when strafing right, negative when strafing left
     */
    public void strafe(double inches) {
        int ticks = ticks(inches);
        motors[2].setTargetPosition(motors[2].getCurrentPosition() + ticks);
        motors[2].setMode(RUN_TO_POSITION);

        setPower(0, 0, STRAFE_SPEED);
        while(opMode.opModeIsActive() && motors[2].isBusy()) {
            setPower(0, 0, STRAFE_SPEED);
        }
        setPower(0);
        motors[2].setMode(RUN_WITHOUT_ENCODER);
    }

    /**
     * rotates a given number of degrees relative to current heading
     * @param degrees negative when rotating clockwise, positive when rotating counterclockwise
     */
    public void rotate(double degrees) {
        double  leftPower, rightPower;
        resetAngle();

        setMode(RUN_WITHOUT_ENCODER, RUN_WITHOUT_ENCODER);

        while(degrees < -180)
            degrees += 360;
        while(degrees > 180)
            degrees -= 360;

        leftPower = degrees > 0 ? -ROTATE_SPEED : ROTATE_SPEED;
        rightPower = degrees > 0 ? ROTATE_SPEED : -ROTATE_SPEED;
        if(degrees == 0)
            return;

        setPower(leftPower, rightPower);

        if (degrees < 0) {
            while (opMode.opModeIsActive() && getAngle() == 0);

            while (opMode.opModeIsActive() && getAngle() > degrees);
        }
        else
            while (opMode.opModeIsActive() && getAngle() < degrees);

        setPower(0);
        resetAngle();
    }

    //----------------------------------------------------------------------------------------------
    // Autonomous PID Methods
    //----------------------------------------------------------------------------------------------

    /**
     * drives a given number of inches, keeping a constant heading through use of a PID
     * controller
     * @param inches positive when driving forwards, negative when driving backwards
     */
    public void drivePID(double inches) {
        int ticks = ticks(inches);
        int leftTarget = motors[0].getTargetPosition() + ticks;
        int rightTarget = motors[1].getTargetPosition() + ticks;

        setTargetPosition(leftTarget, rightTarget);
        setMode(RUN_TO_POSITION, RUN_TO_POSITION);

        resetAngle();

        pidDrive.reset();
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(-DRIVE_SPEED, DRIVE_SPEED);

        while(opMode.opModeIsActive() && motors[0].isBusy() && motors[1].isBusy()) {
            double steer = pidDrive.calculate(getAngle());

            // if driving in reverse, the motor correction also needs to be reversed
            if (inches < 0)
                steer *= -1.0;

            double leftSpeed = DRIVE_SPEED - steer;
            double rightSpeed = DRIVE_SPEED + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            setPower(leftSpeed, rightSpeed);
        }

        setPower(0);
        setMode(RUN_WITHOUT_ENCODER, RUN_WITHOUT_ENCODER);
    }

    /**
     * strafes a given number of inches, keeping a constant heading through use of a PID
     * controller
     * @param inches positive when strafing right, negative when strafing left
     */
    public void strafePID(double inches) {
        int ticks = ticks(inches);
        int target = motors[2].getCurrentPosition() + ticks;

        motors[2].setTargetPosition(target);
        setMode(RUN_WITHOUT_ENCODER, RUN_WITHOUT_ENCODER, RUN_TO_POSITION);

        resetAngle();

        pidStrafe.reset();
        pidStrafe.setSetpoint(0);
        pidStrafe.setOutputRange(-1.0, 1.0);

        while(opMode.opModeIsActive() && motors[0].isBusy() && motors[1].isBusy()) {
            double steer = pidStrafe.calculate(getAngle());

            // if driving in reverse, the motor correction also needs to be reversed
            if(inches < 0)
                steer *= -1.0;

            setPower(steer, -steer, STRAFE_SPEED);
        }

        setPower(0);
        setMode(RUN_WITHOUT_ENCODER, RUN_WITHOUT_ENCODER, RUN_WITHOUT_ENCODER);
    }

    /**
     * performs one cycle of closed loop heading control for the rotatePID() method
     * @return true if error of heading is within tolerable range, false if not
     */
    private boolean onHeading() {
        boolean onTarget = pidRotate.onTarget();

        if(onTarget) {
            setPower(0);
        } else {
            double steer = pidRotate.calculate(getAngle());
            setPower(-ROTATE_SPEED * steer, ROTATE_SPEED * steer);
        }

        return onTarget;
    }

    /**
     * rotates a given number of degrees, doing so by use of a PID controller
     * @param degrees negative when rotating clockwise, positive when rotating counterclockwise
     */
    public void rotatePID(double degrees) {
        resetAngle();
        if (Math.abs(degrees) > 359)
            degrees = (int) Math.copySign(359, degrees);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, ROTATE_SPEED);
        pidRotate.setTolerance(.01);

        while(opMode.opModeIsActive() && !onHeading());
    }
}
