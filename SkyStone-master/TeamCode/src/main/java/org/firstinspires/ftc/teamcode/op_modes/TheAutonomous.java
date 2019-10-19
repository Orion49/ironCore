package org.firstinspires.ftc.teamcode.op_modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name="test auton", group="Linear OpMode")
public class TheAutonomous extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true, true);
        StartingPosition startingPosition = null;

        telemetry.addData("Status", "robot initialized");

        while(!opModeIsActive()) {
            telemetry.addLine("Enter starting position (x = blue building zone, y = red building zone, b = red loading zone, a = blue loading zone)");

            if(gamepad1.x)
                startingPosition = StartingPosition.BLUE_BUILDING_ZONE;
            if(gamepad1.y)
                startingPosition = StartingPosition.RED_BUILDING_ZONE;
            if(gamepad1.b)
                startingPosition = StartingPosition.RED_LOADING_ZONE;
            if(gamepad1.a)
                startingPosition = StartingPosition.BLUE_LOADING_ZONE;

            telemetry.addData("Starting Position", startingPosition != null ? startingPosition : "null");
            telemetry.update();
        }

        switch(startingPosition) {
            case BLUE_BUILDING_ZONE:
//                robot.driveTrain.strafe(9.4);
                robot.driveTrain.drive(-27.2);
                //deploy hook and grab onto foundation
                robot.driveTrain.drive(27.2);
                robot.driveTrain.strafe(-81.4);
                robot.driveTrain.drive(-18);
                //detect which stone is skystone and return to same position
                robot.driveTrain.strafe(49.0);
                //drop skystone
                robot.driveTrain.strafe(-72.4);
                //detect which stone is skystone and return to same position
                robot.driveTrain.strafe(72.4);
                //drop skystone
                robot.driveTrain.strafe(-12.8);
                break;
            case RED_BUILDING_ZONE:
                robot.driveTrain.strafe(-9.4);
                robot.driveTrain.drive(-27.2);
                //deploy hook and grab onto foundation
                robot.driveTrain.drive(27.2);
                robot.driveTrain.strafe(81.4);
                robot.driveTrain.drive(-18);
                //detect which stone is skystone and return to same position
                robot.driveTrain.strafe(-49.0);
                //drop skystone
                robot.driveTrain.strafe(72.4);
                //detect which stone is skystone and return to same position
                robot.driveTrain.strafe(-72.4);
                //drop skystone
                robot.driveTrain.strafe(12.8);
                break;
        }
    }

    enum StartingPosition {
        BLUE_BUILDING_ZONE, RED_BUILDING_ZONE, RED_LOADING_ZONE, BLUE_LOADING_ZONE
    }
}
