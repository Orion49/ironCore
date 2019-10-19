package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

import java.util.HashMap;
import java.util.Map;

public class Arm extends Subsystem {
    public DcMotor[] motors; //0 - leftRotate, 1 - rightRotate
    private String[] motorNames = {"left_rotate_drive", "right_rotate_drive", "spool_drive"};

    //----------------------------------------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------------------------------------

    public Arm(LinearOpMode opMode, boolean autonomous) {
        super(opMode, autonomous);

        motors = new DcMotor[motorNames.length];
        for(int i = 0; i < motorNames.length; i++) {
            motors[i] = opMode.hardwareMap.get(DcMotor.class, motorNames[i]);
            motors[i].setDirection(i == 0 ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        opMode.telemetry.addData("Status", "Arm instantiated");
        opMode.telemetry.update();
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry
    //----------------------------------------------------------------------------------------------

    @Override
    public Map<String, Object> updateTelemetry() {
        Map<String, Object> telemetryData = new HashMap<>();
        
        for(int i = 0; i < motors.length; i++) {
            telemetryData.put(motorNames[i] + " encoder counts", motors[i].getCurrentPosition());
            telemetryData.put(motorNames[i] + " power", motors[i].getPower());
        }

        return telemetryData;
    }

    //----------------------------------------------------------------------------------------------
    // Methods
    //----------------------------------------------------------------------------------------------

    public void setPower(double... powers) {
        for(int i = 0; i < powers.length; i++)
            motors[i].setPower(powers[i]);
    }

    public void setMode(DcMotor.RunMode... modes) {
        for(int i = 0; i < modes.length; i++)
            motors[i].setMode(modes[i]);
    }

    public void setTargetPosition(double... targetPositions) {
        for(int i = 0; i < targetPositions.length; i++)
            motors[i].setTargetPosition((int) Math.round(targetPositions[i]));
    }

    public void rotateUp() {
        setPower(-ROTATE_SPEED, -ROTATE_SPEED);
    }

    public void rotateDown() {
        setPower(ROTATE_SPEED, ROTATE_SPEED);
    }

    public void extend() {
        motors[2].setPower(ARM_EXTEND_POWER);
    }

    public void retract() {
        motors[2].setPower(-ARM_EXTEND_POWER);
    }

    public void rotateUp(double ticks) {
        setTargetPosition(-ticks, -ticks);
        setMode(RUN_TO_POSITION, RUN_TO_POSITION);
        setPower(-ROTATE_SPEED, -ROTATE_SPEED);
        while(motors[1].isBusy() && motors[2].isBusy()) {
            setPower(-ROTATE_SPEED, -ROTATE_SPEED);
        }
        setMode(RUN_WITHOUT_ENCODER);
    }

    public void rotateDown(double ticks) {
        setTargetPosition(ticks, ticks);
        setMode(RUN_TO_POSITION, RUN_TO_POSITION);
        setPower(ROTATE_SPEED, ROTATE_SPEED);
        while(motors[1].isBusy() && motors[2].isBusy()) {
            setPower(ROTATE_SPEED, ROTATE_SPEED);
        }
        setMode(RUN_WITHOUT_ENCODER);
    }
}
