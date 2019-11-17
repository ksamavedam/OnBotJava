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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.lang.Math;

public class RobotSense {
/*    
    RobotHardware hw;

    enum Zone { LOADING, BUILDING};
    // initialize later
RobotSense(RobotHardware rhw) {
    hw = rhw; 
    initTfod();
    initVuforia();
}

    // give what zone the robot is in (use the getCoordinates )
    public void getZone() {

    }

    // boolean for whether skystone is in sight; t = sky, f = no
    public boolean isSkystone() {

    }

    // detect skystone and determine relative position in inches
    public double[] locateSkystone() {

    }

    // returns angle using imu
    public double imuAngle() {

    }

    // returns coordinates to getZone to determine what side we're on
    private double getCoordinates() {

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

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id",
                hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
*/
}