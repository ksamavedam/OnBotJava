package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.lang.Math;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class RobotSense {
    RobotHardware hw;

    enum Zone {
        LOADING, BUILDING
    }

    // initialize later
    RobotSense(RobotHardware rhw) {
        hw = rhw;
    }

    // give what zone the robot is in (use the getCoordinates )
    public Zone getZone() {

        return(Zone.LOADING); 
    }

    // boolean for whether skystone is in sight; t = sky, f = no
    public boolean isSkystone() {
        boolean var=true;//tba later
        return var;
    }

    // detect skystone and determine relative position in inches
    public double[] locateSkystone() {
        double[] var={0,0};
        return var;
    }

    // returns angle using imu
    public double imuAngle() {
        double var=0;
        return var;
    }

    // returns coordinates to getZone to determine what side we're on
    private double[] getCoordinates() {
        double[] var={0,0,0};
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