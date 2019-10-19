package org.firstinspires.ftc.teamcode.util;
import android.os.Environment;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.*;
import java.io.*;

/** Controls:
 * y - drive forward 5 inches
 * a - drive reverse 5 inches
 * dpad_up - drive forward 20 inches
 * dpad_down - drive reverse 20 inches
 *
 * b - strafe right 5 inches
 * x - strafe left 5 inches
 * dpad_right - strafe right 20 inches
 * dpad_left - strafe left 20 inches
 *
 * right_bumper - rotate clockwise 90 degrees
 * left_bumper - rotate counterclockwise 90 degrees
 * right_stick_button - rotate clockwise 10 degrees
 * left_stick_button - rotate counterclockwise 10 degrees
 *
 * guide - stop recording
 */


public class AutonomousRecorder {
    private PrintWriter out;
    private Robot robot;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private StickyGamepad stickyGamepad;

    //----------------------------------------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------------------------------------

    public AutonomousRecorder(Robot robot) {
        this.robot = robot;
        telemetry = robot.opMode.telemetry;
        gamepad = robot.opMode.gamepad1;
        stickyGamepad = new StickyGamepad(robot.opMode.gamepad1);
    }

    //----------------------------------------------------------------------------------------------
    // Record And Play Methods
    //----------------------------------------------------------------------------------------------

    public void record(String filepath) {
        telemetry.addData("Status", "recording \"" + filepath + "\"");
        telemetry.update();

        try {
            out = new PrintWriter(new FileWriter(Environment.getExternalStorageDirectory().getName() + filepath + ".txt"));
        } catch (IOException exception) {
            telemetry.addData("Status", "IOException thrown: " + exception);
            telemetry.update();
        }

        List<String> strings = new ArrayList<>();

        while(!gamepad.guide) {
            if(stickyGamepad.y) {
                robot.driveTrain.drive(5);
                strings.add("drive 5");
            } else if(stickyGamepad.a) {
                robot.driveTrain.drive(-5);
                strings.add("drive -5");
            } else if(stickyGamepad.dpad_up) {
                robot.driveTrain.drive(20);
                strings.add("drive 20");
            } else if(stickyGamepad.dpad_down) {
                robot.driveTrain.drive(-20);
                strings.add("drive -20");
            } else if(stickyGamepad.b) {
                robot.driveTrain.strafe(5);
                strings.add("strafe 5");
            } else if(stickyGamepad.x) {
                robot.driveTrain.strafe(-5);
                strings.add("strafe -5");
            } else if(stickyGamepad.dpad_right) {
                robot.driveTrain.strafe(20);
                strings.add("strafe 20");
            } else if(stickyGamepad.dpad_left) {
                robot.driveTrain.strafe(-20);
                strings.add("strafe -20");
            } else if(stickyGamepad.right_bumper) {
                robot.driveTrain.rotate(-90);
                strings.add("rotate -90");
            } else if(stickyGamepad.left_bumper) {
                robot.driveTrain.rotate(90);
                strings.add("rotate 90");
            } else if(stickyGamepad.right_stick_button) {
                robot.driveTrain.rotate(-10);
                strings.add("rotate -10");
            } else if(stickyGamepad.left_stick_button) {
                robot.driveTrain.rotate(10);
                strings.add("rotate 10");
            }
        }

        telemetry.addData("Status", "writing");
        telemetry.update();

        condenseStrings(strings);

        for(String str : strings)
            out.println(str);

        telemetry.addData("Status", "finished recording and writing \"" + filepath + "\"");
        telemetry.update();
    }

    public void play(String filepath) {
        telemetry.addData("Status", "playing \"" + filepath + "\"");
        telemetry.update();

        try {
            Scanner in = new Scanner(new BufferedReader(new FileReader(Environment.getExternalStorageDirectory().getName() + filepath + ".txt")));

            while(in.hasNextLine()) {
                String line = in.nextLine();
                if(line.split(" ")[0].equals("drive"))
                    robot.driveTrain.drive(Double.parseDouble(line.split(" ")[1]));
            }
        } catch (IOException exception) {
            telemetry.addData("Status", "IOException thrown: " + exception);
            telemetry.update();
        }

        telemetry.addData("Status", "finished playing \"" + filepath + "\"");
        telemetry.update();
    }

    //----------------------------------------------------------------------------------------------
    // Helper Methods
    //----------------------------------------------------------------------------------------------

    private void condenseStrings(List<String> strings) {
        int tempIndex = -1;
        String tempValue = "";
        for(int i = 0; i < strings.size(); i++) {
            String[] currentSplit = strings.get(i).split(" "), tempSplit = tempValue.split(" ");
            if(currentSplit[0].equals(tempSplit[0])) {
                strings.set(tempIndex, tempSplit[0] + " " + (Double.parseDouble(tempSplit[1]) + Double.parseDouble(currentSplit[1])));
                strings.remove(i);
            } else {
                tempIndex = i;
                tempValue = strings.get(i);
            }
        }

        for(int i = 0; i < strings.size(); i++)
            if(Double.parseDouble(strings.get(i).split(" ")[1]) == 0)
                strings.remove(i);
    }
}