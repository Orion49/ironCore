package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.HashMap;
import java.util.Map;

public class Hook extends Subsystem {

    public CRServo hookServo;
    private String hookServoName = "hook_servo";

    //----------------------------------------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------------------------------------

    public Hook(LinearOpMode opMode, boolean autonomous) {
        super(opMode, autonomous);
        hookServo = opMode.hardwareMap.get(CRServo.class, hookServoName);

        opMode.telemetry.addData("Status", "Hook instantiated");
        opMode.telemetry.update();
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry
    //----------------------------------------------------------------------------------------------

    @Override
    public Map<String, Object> updateTelemetry() {
        Map<String, Object> telemetryData = new HashMap<>();

        telemetryData.put("hook_servo power", hookServo.getPower());

        return telemetryData;
    }

    //----------------------------------------------------------------------------------------------
    // Methods
    //----------------------------------------------------------------------------------------------

    public void deploy() {
        hookServo.setPower(1.0);
    }

    public void retract() {
        hookServo.setPower(-1.0);
    }
}
