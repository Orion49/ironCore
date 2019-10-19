package org.firstinspires.ftc.teamcode.op_modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.util.StickyGamepad;

import static org.firstinspires.ftc.teamcode.util.Constants.*;

/**
 * Controls:
 * left_stick_x - strafing
 * left_stick_y - driving
 * right_stick_x - turning
 * y - set scale to 1.0
 * b - set scale to 0.75
 * a - set scale to 0.5
 * x - set scale to 0.25
 * dpad_up - increase smoothing factor by 0.1
 * dpad_down - decrease smoothing factor by 0.1
 * dpad_right - increase strafe smoothing factor by 0.1
 * dpad_left - decrease strafe smoothing factor by 0.1
 */

@TeleOp(name="the teleop", group="Linear OpMode")
public class TheTeleOp extends LinearOpMode {

    private Robot robot;
    private ExponentialSmoother smootherLeft, smootherRight, smootherStrafe;
    private StickyGamepad stickyGamepad;
    private double scale;

    public void runOpMode() {
        robot = new Robot(this, false, true);

        DRIVE_SMOOTHING_FACTOR = 0.79;
        STRAFE_SMOOTHING_FACTOR = 0.79;

        smootherLeft = new ExponentialSmoother(DRIVE_SMOOTHING_FACTOR);
        smootherRight = new ExponentialSmoother(DRIVE_SMOOTHING_FACTOR);
        smootherStrafe = new ExponentialSmoother(STRAFE_SMOOTHING_FACTOR);

        stickyGamepad = new StickyGamepad(gamepad1);

        scale = 1.0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            double leftPower  = (-gamepad1.left_stick_y + gamepad1.right_stick_x) * scale;
            double rightPower = (-gamepad1.left_stick_y - gamepad1.right_stick_x) * scale;
            double strafePower = gamepad1.left_stick_x * scale;

            robot.driveTrain.setPower(
                    smootherLeft.update(Range.clip(leftPower, -1, 1)),
                    smootherRight.update(Range.clip(rightPower, -1, 1)),
                    smootherStrafe.update(Range.clip(strafePower, -1, 1))
            );

            if(stickyGamepad.y)
                scale += scale == 1.0 ? -0.75 : 0.25;

            if(gamepad1.right_bumper)
                robot.arm.rotateDown();
            else if(gamepad1.left_bumper)
                robot.arm.rotateUp();
            else
                robot.arm.setPower(0, 0);

            if(gamepad1.dpad_up)
                robot.arm.extend();
            else if(gamepad1.dpad_down)
                robot.arm.retract();
            else
                robot.arm.motors[2].setPower(0);

            if(gamepad1.dpad_right)
                robot.hook.deploy();
            else if(gamepad1.dpad_left)
                robot.hook.retract();
            else
                robot.hook.hookServo.setPower(0);

            smootherLeft.setSmoothingFactor(DRIVE_SMOOTHING_FACTOR);
            smootherRight.setSmoothingFactor(DRIVE_SMOOTHING_FACTOR);
            smootherStrafe.setSmoothingFactor(STRAFE_SMOOTHING_FACTOR);

            robot.updateTelemetry();

            robot.hook.hookServo.setPower(0);
            robot.arm.setPower(0, 0, 0);

            telemetry.addData("Smoothing Factors", "drive: %f, strafe: %f", DRIVE_SMOOTHING_FACTOR, STRAFE_SMOOTHING_FACTOR);
            telemetry.update();
            stickyGamepad.update();
        }
    }
}
