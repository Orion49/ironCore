package org.firstinspires.ftc.teamcode.op_modes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.AutonomousRecorder;

/** Controls
 * guide - record test
 * a - play test
 */

@TeleOp(name = "Recorded Autonomous", group = "Linear OpMode")
public class RecordedAutonomous extends LinearOpMode {
    private Robot robot;
    private AutonomousRecorder recorder;

    @Override
    public void runOpMode() {
        robot = new Robot(this, true, false);
        recorder = new AutonomousRecorder(robot);

        telemetry.addData(">", "press 'guide' to begin recording");
        telemetry.addData(">", "press 'a' to play the recording");
        telemetry.update();

        while(opModeIsActive()) {
            if(gamepad1.guide)
                recorder.record("Test");
            if(gamepad1.a)
                recorder.play("Test");
        }
    }
}