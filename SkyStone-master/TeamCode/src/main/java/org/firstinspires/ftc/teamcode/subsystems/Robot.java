package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Robot {

    public LinearOpMode opMode;

    public DriveTrain driveTrain;
    public SkystoneDetector skystoneDetector;
    public Arm arm;
    public Hook hook;

    public List<Subsystem> subsystems;

    public FtcDashboard dashboard;

    //----------------------------------------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------------------------------------

    public Robot(LinearOpMode opMode, boolean autonomous, boolean useDashboard) {
        this.opMode = opMode;

        if(useDashboard)
            dashboard = FtcDashboard.getInstance();

        driveTrain = new DriveTrain(opMode, autonomous);
        skystoneDetector = new SkystoneDetector(opMode, autonomous);
        arm = new Arm(opMode, autonomous);
        hook = new Hook(opMode, autonomous);
        if(autonomous)
            skystoneDetector.activate();

        subsystems = new ArrayList<>();
        subsystems.add(driveTrain);
        if(autonomous) {
            subsystems.add(skystoneDetector);
        }
        subsystems.add(arm);
        subsystems.add(hook);

        opMode.telemetry.addData("Status", "robot initialized");
        opMode.telemetry.update();
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry
    //----------------------------------------------------------------------------------------------

    public void updateDashboard() {
        TelemetryPacket packet = new TelemetryPacket();

        for(Subsystem subsystem: subsystems)
            packet.putAll(subsystem.updateTelemetry());

        dashboard.sendTelemetryPacket(packet);
    }

    public void updateTelemetry() {
        Map<String, Object> telemetryData = new HashMap<String, Object>();

        for(Subsystem subsystem: subsystems)
            telemetryData.putAll(subsystem.updateTelemetry());

        for(Map.Entry<String, Object> entry: telemetryData.entrySet())
            opMode.telemetry.addData(entry.getKey(), entry.getValue());
        opMode.telemetry.update();
    }
}
