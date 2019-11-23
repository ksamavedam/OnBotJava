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

public class RobotSense {

    RobotHardware hw;
    Telemetry telemetry;
    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;
    double dist;
    private static final int max_tf_iterations = 100;  


    enum Zone {
        LOADING, BUILDING
    };

    enum Direction {
        LEFT, RIGHT, NONE
    };

    public class DirectionVals {
        RobotSense.Direction dir;
        double distance;
        double angle;

        DirectionVals() {
            dir = Direction.NONE;
            distance = 0;
            angle = 0;
        }
    };

    // initialize later
    RobotSense(RobotHardware rhw, Telemetry t) {
        hw = rhw;
        telemetry = t;
        dist = 0;
        hw.initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            hw.initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        tfod = hw.tfod;
        vuforia = hw.vuforia;

        if (tfod != null) {
            tfod.activate();
        }

    }

    void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    // give what zone the robot is in (use the getCoordinates )
    public void getZone() {

    }

    // Some how this is not working; do not use this
    // use locateSkystone instead
    public boolean isSkystoneInSight() {
        return false;
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
                telemetry.update();
            }
        }

    }

    // detect skystone and determine relative position in inches
    // Pass iteration count which is number of detection attempts should be done
    // Return array
    // index 0: 1-detected or 0-not
    // index 1 - Distance
    // index 2 - Angle
    public double[] locateSkystone() {
        double[] dist_angle = new double[3]; //(whether skystone is detected or not, diag move, angle to object)
        double detected = 0;
        int count = 0;
        while (count++ < max_tf_iterations) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {

                    for (Recognition recognition : updatedRecognitions) {
                        double d_angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                        if (recognition.getLabel() == "Skystone") {
                            dist_angle[2] = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                            detected = 1;
                            break; // as soon as we detect, break and return the results
                        }
                    }
                }
            }
        }
        dist_angle[0] = detected;
        dist_angle[1] = calcDiagMove(dist_angle[2], getDistance());

        return dist_angle;
    }

    public double getDistance() {
        //return dist++;
        return hw.sensorRange.getDistance(DistanceUnit.INCH);
    }

    // returns angle using imu
    public double imuAngle() {
        double var = 0;
        return var;
    }

    // returns coordinates to getZone to determine what side we're on
    private double[] getCoordinates() {
        double[] var = { 0, 0, 0 };
        return var;
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