package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import static org.firstinspires.ftc.teamcode.util.Constants.*;

public class SkystoneDetector extends Subsystem {

    private OpenGLMatrix lastLocation;
    public VuforiaLocalizer vuforia;
    private VuforiaTrackables targetsSkystone;
    private VuforiaTrackable stoneTarget;

    private boolean skystoneVisible;
    public SkystonePosition skystonePosition;
    private float phoneXRotate, phoneYRotate, phoneZRotate;
    private double[] rotation, translation;

    //----------------------------------------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------------------------------------

    public SkystoneDetector(LinearOpMode opMode, boolean autonomous) {
        super(opMode, autonomous);

        if(autonomous) {
            int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
            parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            targetsSkystone = this.vuforia.loadTrackablesFromAsset("Skystone");
            stoneTarget = targetsSkystone.get(0);
            stoneTarget.setName("Stone Target");

            // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
            // Rotated it to to face forward, and raised it to sit on the ground correctly.
            // This can be used for generic target-centric approach algorithms
            stoneTarget.setLocation(OpenGLMatrix
                    .translation(0, 0, STONE_Z)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

            // We need to rotate the camera around it's long axis to bring the correct camera forward.
            phoneYRotate = -90;

            //translating the camera lens to where it is on the robot.
            OpenGLMatrix robotFromCamera = OpenGLMatrix
                    .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                    .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

            ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);

            rotation = new double[3];
            translation = new double[3];

            opMode.telemetry.addData("Status", "SkystoneDetector instantiated");
            opMode.telemetry.update();
        }
    }

    //----------------------------------------------------------------------------------------------
    // Telemetry
    //----------------------------------------------------------------------------------------------

    @Override
    public Map<String, Object> updateTelemetry() {
        Map<String, Object> telemetryData = new HashMap<>();

        telemetryData.put("Skystone Visible", skystoneVisible);
        telemetryData.put("Translation (in)", String.format("{X, Y, Z} = %.1f, %.1f, %.1f", translation[0], translation[1], translation[2]));
        telemetryData.put("Rotation (deg)", String.format("{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation[0], rotation[1], rotation[2]));
        telemetryData.put("Skystone Position", skystonePosition != null ? skystonePosition : "null");

        return telemetryData;
    }

    //----------------------------------------------------------------------------------------------
    // Methods
    //----------------------------------------------------------------------------------------------

    public void detectSkystone() {
        if (((VuforiaTrackableDefaultListener) stoneTarget.getListener()).isVisible()) {
            opMode.telemetry.addData("Skystone Visible", true);
            skystoneVisible = true;

            // getUpdatedRobotLocation() will return null if no new information is available since
            // the last time that call was made, or if the skystone is not currently visible.
            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) stoneTarget.getListener()).getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
            }

            // express position (translation) of robot in inches.
            VectorF trans = lastLocation.getTranslation();
            translation = new double[] {trans.get(0) / MM_PER_INCH, trans.get(1) / MM_PER_INCH, trans.get(2) / MM_PER_INCH};
            opMode.telemetry.addData("Translation (in)", "{X, Y, Z} = %.1f, %.1f, %.1f", translation[0], translation[1], translation[2]);

            // express skystone position using translation in the x direction
            if(translation[0] < SKYSTONE_LEFT_BOUND)
                skystonePosition = SkystonePosition.LEFT;
            else if(translation[0] < SKYSTONE_RIGHT_BOUND)
                skystonePosition = SkystonePosition.CENTER;
            else
                skystonePosition = SkystonePosition.RIGHT;

            opMode.telemetry.addData("Skystone Position", skystonePosition);

            // express the rotation of the robot in degrees.
            Orientation rot = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            rotation = new double[] {rot.firstAngle, rot.secondAngle, rot.thirdAngle};
            opMode.telemetry.addData("Rotation (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation[0], rotation[1], rotation[2]);
        } else {
            opMode.telemetry.addData("Skystone Visible", false);
        }
        opMode.telemetry.update();
    }

    public void activate() {
        targetsSkystone.activate();
    }

    public void deactivate() {
        targetsSkystone.deactivate();
    }

    enum SkystonePosition {
        LEFT, CENTER, RIGHT
    }
}
