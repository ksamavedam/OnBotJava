package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.lang.Math;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class RobotSense {

    RobotHardware hw;
    Telemetry telemetry;
    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;
    double dist;
    private static final int max_tf_iterations = 100;
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    public VuforiaTrackables targetsSkyStone = null;
    List<VuforiaTrackable> allTrackables = null;
    OpenGLMatrix lastLocation = null;
    float mmPerInch = 25.4f;
    float mmBotWidth = 18 * mmPerInch; // ... or whatever is right for your robot
    float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch; // the FTC field is ~11'10" center-to-center of the glass panels

    enum Zone {
        LOADING, BUILDING
    };

    public class SSLocation {
        boolean detected;
        double diagDistance;
        double hzDistance;
        double angle;

        SSLocation() {
            detected = false;
            diagDistance = 0;
            hzDistance = 0;
            angle = 0;
        }
        
    };
    public class VuCoordinates {
        boolean detected;
        String  targetName;
        double posX;
        double posY;
        double posZ;

        double rotRoll;
        double rotPitch;
        double rotHeading;
        String retStr;
        
        VuCoordinates(){
            detected = false;
            posX = posY = posZ = 0;
            rotRoll = rotPitch = rotHeading = 0;
            retStr = "";
        }

    };

    // initialize later
    RobotSense(RobotHardware rhw, Telemetry t, boolean tfodEnable) {
        hw = rhw;
        telemetry = t;
        dist = 0;
        hw.initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector() && tfodEnable) {
            hw.initTfod();
            tfod = hw.tfod;
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        vuforia = hw.vuforia;
        initVuSkystone();
        if (tfod != null) {
            tfod.activate();
        }

    }

    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void initVuSkystone() {
        final float mmPerInch = 25.4f;
        final float mmTargetHeight = (6) * mmPerInch; // the height of the center of the target image above the floor

        // Constant for Stone Target
        final float stoneZ = 2.00f * mmPerInch;

        // Constants for the center support targets
        final float bridgeZ = 6.42f * mmPerInch;
        final float bridgeY = 23 * mmPerInch;
        final float bridgeX = 5.18f * mmPerInch;
        final float bridgeRotY = 59; // Units are degrees
        final float bridgeRotZ = 180;

        // Constants for perimeter targets
        final float halfField = 72 * mmPerInch;
        final float quadField = 36 * mmPerInch;

        float phoneXRotate = 0;
        float phoneYRotate = -90; // back camera
        float phoneZRotate = 0;
        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of
        // the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 4.0f * mmPerInch; // eg: Camera is 4 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch; // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0; // eg: Camera is ON the robot's center line
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        // For convenience, gather together all the trackable objects in one
        // easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);
        // Set the position of the Stone Target. Since it's not fixed in position,
        // assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix.translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        // Set the position of the bridge support targets with relation to origin
        // (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix.translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix.translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix.translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix.translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        // Set the position of the perimeter targets with relation to origin (center of
        // field)
        red1.setLocation(OpenGLMatrix.translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix.translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix.translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix.translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix.translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix.translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix.translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix.translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate,
                        phoneXRotate));

        /** Let all the trackable listeners know where the phone is. */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera,
                    VuforiaLocalizer.CameraDirection.BACK);
        }

    }

    // give what zone the robot is in (use the getCoordinates )
    public void getZone() {

    }

    // Test function - not used anywhere
    public void detectSkystone() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available
            // since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(),
                            recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(),
                            recognition.getBottom());
                }
                // telemetry.update();
            }
        }

    }

    // detect skystone and determine relative position in inches

    public SSLocation locateSkystone() {
        SSLocation ssl = new SSLocation();
        int count = 0;
        while (count++ < max_tf_iterations) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {

                    for (Recognition recognition : updatedRecognitions) {
                        double d_angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                        if (recognition.getLabel() == "Skystone") {
                            ssl.angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                            ssl.detected = true;
                            break; // as soon as we detect, break and return the results
                        }
                    }
                }
            }
        }
        ssl.diagDistance = calcDiagMove(ssl.angle, getDistance());
        ssl.hzDistance = calcHorizMove(ssl.angle, getDistance());
        return ssl;
    }

    public double getDistance() {
        // return dist++;
        return hw.sensorRange.getDistance(DistanceUnit.INCH);
    }

    // returns angle using imu
    public double imuAngle() {
        double var = 0;
        return var;
    }

    // returns coordinates to getZone to determine what side we're on
    public VuCoordinates getCoordinates() {
        VuCoordinates vc = new VuCoordinates();
        int count = 0;
        int iterations = 100;
        vc.retStr = "None";
        targetsSkyStone.activate();
        while(iterations-- > 0){
            // check all the trackable targets to see which one (if any) is visible.
            boolean targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    targetVisible = true;
                    count++;
                    vc.detected = true;
                    vc.retStr = String.format("Visible Target: %s Detected Count:%d\n", trackable.getName(), count);
                    vc.targetName = trackable.getName();
                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                vc.retStr += String.format("POS(in) {X, Y, Z} = %.1f, %.1f, %.1f", translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
                vc.retStr +=  String.format("Rot (deg) {Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                vc.posX = translation.get(0) / mmPerInch;
                vc.posX = translation.get(0) / mmPerInch;
                vc.posX = translation.get(0) / mmPerInch;

                vc.rotRoll = rotation.firstAngle;
                vc.rotPitch = rotation.secondAngle;
                vc.rotHeading = rotation.thirdAngle;

                
            }
    
            // Disable Tracking when we are done;
            if(targetVisible)
            {
                targetsSkyStone.deactivate();
                return vc;
            }
        }
        return vc;
    }

    public double calcHorizMove(double d_angle, double distance) {
        double r_angle = Math.toRadians(d_angle);
        double horiz_move = (distance * Math.tan(r_angle));
        return horiz_move;
    }

    public double calcDiagMove(double d_angle, double distance) {
        double r_angle = Math.toRadians(d_angle);
        double diag_move = (distance / Math.cos(r_angle));
        return diag_move;
    }
}