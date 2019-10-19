package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Map;

public abstract class Subsystem {

    protected LinearOpMode opMode;
    protected boolean autonomous;

    protected Subsystem(LinearOpMode opMode, boolean autonomous) {
        this.opMode = opMode;
        this.autonomous = autonomous;
    }

    public abstract Map<String, Object> updateTelemetry();
}
