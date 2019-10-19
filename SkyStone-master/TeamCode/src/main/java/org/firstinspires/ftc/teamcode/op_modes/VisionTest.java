package org.firstinspires.ftc.teamcode.op_modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.util.Constants.VUFORIA_LICENSE_KEY;

@Autonomous(name="Vision Test", group="Tests")
public class VisionTest extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        msStuckDetectStop = 2500;
        robot = new Robot(this, true, true);
        robot.dashboard.startCameraStream(robot.skystoneDetector.vuforia, 0);
        waitForStart();
        while(opModeIsActive()) {
            robot.updateTelemetry();
            robot.skystoneDetector.detectSkystone();
        }

        robot.skystoneDetector.deactivate();
    }
}
