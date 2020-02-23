package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Blinker;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import java.util.Arrays;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RobotDrive.Direction;

import java.util.List;
import java.lang.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoTest", group = "Linear Opmode")
public class AutoTest extends LinearOpMode {
    RobotHardware hw = null;
    private RobotDrive rd = null;
    private RobotSense rs = null;
    //private static final String robotName = "VuforiaTest";
     private static final String robotName = "OtterMelon";
    private static final double initialRDist = 31.5;
    private static final double toMoveRDist = 25;


    int ssDetCount = 0;
    int ssNotDetCount = 0;
    ElapsedTime mRunTime;

    @Override
    public void runOpMode() {
        // These must be initialized in the runOpmode
        hw = new RobotHardware(robotName, hardwareMap);
         rd = new RobotDrive(hw);
         rs=new RobotSense(hw, telemetry);
         /*rd.moveDist(RobotDrive.Direction.FORWARD, .5, .3);
         rd.moveDist(RobotDrive.Direction.REVERSE, .5, .3);
         rd.moveDist(RobotDrive.Direction.FORWARD, .5, 1);
         rd.moveDist(RobotDrive.Direction.REVERSE, .5, 1);*/
        telemetry.addData("Ready! ", "Go Flamangos!"); 
        telemetry.update();

        hw.f_servoLeft.setPosition(1);
        hw.f_servoRight.setPosition(0.5);
        
        rd.moveArm(hw.startPos());
        hw.gripper.setPosition(.8);
        waitForStart();
        while (opModeIsActive()) {

            //rd.proportionalTurn(90);
            rd.moveDist(RobotDrive.Direction.FORWARD, 10, 1);
            moveCorrect(Math.toRadians(270), 1, 0, 1.5);
            rd.proportionalTurn(180);
            
            telemetry.update();            
            break;
        }
    }

    public void moveCorrect(double angleOfMove, double power, double targetHeading, double time ){

        Orientation angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double[] powers = new double[4];
        ElapsedTime mRuntime = new ElapsedTime();
        while(mRuntime.time() < time){
            powers = move(angleOfMove, power, propPower(targetHeading, angles));
            hw.tlMotor.setPower(powers[0]);
            hw.blMotor.setPower(powers[1]); 
            hw.brMotor.setPower(powers[2]);
            hw.trMotor.setPower(powers[3]);
            telemetry.addData("In loop", powers[0]);
            telemetry.update();
            }      

        rd.proportionalTurn(0,.25);
        
    }

    private double[] move(double angle, double scale, double turnScale) {
        double topLeft = 0;
        double bottomLeft = 0;
        double bottomRight = 0;
        double topRight = 0;
        double maxPower = 0;
        double[] powers = new double[4];

        if (scale != 0.0) {
            topLeft = (Math.cos(angle)) + -1 * (Math.sin(angle)) + turnScale * -1;
            bottomLeft = (Math.cos(angle)) + (Math.sin(angle)) + turnScale * -1;
            bottomRight = (Math.cos(angle)) + -1 * (Math.sin(angle)) + turnScale * 1;
            topRight = (Math.cos(angle)) + (Math.sin(angle)) + turnScale * 1;

            maxPower = getMaxPower(topLeft, bottomLeft, bottomRight, topRight);

            topLeft = topLeft / maxPower;
            bottomLeft = bottomLeft / maxPower;
            bottomRight = bottomRight / maxPower;
            topRight = topRight / maxPower;

            topLeft *= scale;
            bottomLeft *= scale;
            bottomRight *= scale;
            topRight *= scale;

            powers[0] = topLeft;
            powers[1] = bottomLeft;
            powers[2] = bottomRight;
            powers[3] = topRight;
        } else {
            powers[0] = -1 * turnScale;
            powers[1] = -1 * turnScale;
            powers[2] = 1 * turnScale;
            powers[3] = 1 * turnScale;

        }
        return powers;
    }

    private double getMaxPower(double a, double b, double c, double d) {

        a = Math.abs(a);
        b = Math.abs(b);
        c = Math.abs(c);
        d = Math.abs(d);
        return (Math.max(a, Math.max(b, Math.max(c, d))));

    }

    private double propPower(double targetAngle, Orientation angles){

        angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double delta = targetAngle - angles.firstAngle;
        telemetry.addData("current angle: ", angles.firstAngle);
        telemetry.addData("delts: ", delta);
        //if(delta!=0){

        if(delta>180){

            delta = delta - 360;
        }
        else if(delta <-180){

            delta += 360;
        }
        double power = (delta * .025);
        power = Math.max((Math.min(Math.abs(power), 75)),.15) * (Math.abs(power) / power) ;
        return power;
    }


    
}
