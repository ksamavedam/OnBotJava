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

@Autonomous(name = "Turning Test", group = "Linear Opmode")
public class TurningTests extends LinearOpMode {
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

        /*hw.armRight.setPosition(1-.3);
        hw.armLeft.setPosition(.3);
        hw.level.setPosition(.3+.05);*/
        hw.f_servoLeft.setPosition(1);
        hw.f_servoRight.setPosition(0.5);
        
        rd.moveArm(hw.startPos());
        hw.gripper.setPosition(0.8);
        waitForStart();
        while (opModeIsActive()) {
            double sumAngle = 0;
            double rotations = 0;

            proportionalTurn(90, 3, sumAngle);
            sleep(1500);
            break;
        }
    }
    public void proportionalTurn(double targetChange, double time, double& sumAngle, double& rotations){
        Orientation angles= hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        mRunTime= new ElapsedTime();

        double proportionalPower = 1;
        double delta = 0;
        double angle = sumAngle;
        double targetAngle = sumAngle + targetChange; 
        mRunTime().reset;
        while (mRunTime.time() < time){
            angles= hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double lastAngle = angle;
            angle= angles.firstAngle % 360;
            delta = angle-lastAngle;
            
            if (Math.abs(delta) > 350){
                if (delta > 0)
                    rotations--;
                else if (delta < 0)
                    rotations++;
            }
            
            sumAngle = 360*rotations+angle;
            
            proportionalPower = (targetAngle - sumAngle)*.025;
            proportionalPower = (Math.min(Math.abs(proportionalPower), .75)) * (Math.abs(proportionalPower) / proportionalPower); 
            //set the powers with appropriate signs
        }

        setPower(0, 0, 0, 0);
    }
}
