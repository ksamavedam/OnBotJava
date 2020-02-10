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

@Autonomous(name = "BlueFoundationFar", group = "Linear Opmode")
public class BlueFoundationFar extends LinearOpMode {
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

        rd.moveArm(hw.startPos());
        hw.gripper.setPosition(0);
        waitForStart();
        while (opModeIsActive()) {

            BuildingZone();
            break;
        }

        rs.shutdown();
    }


    
    public void BuildingZone() {

        //set gripper so it doesn't run into bridge
        hw.gripper.setPosition(.3);

        //lock foundation
        rd.moveDist(RobotDrive.Direction.REVERSE, 20, .75);
        rd.moveDist(RobotDrive.Direction.RIGHT, 12, .75);
        rd.moveDist(RobotDrive.Direction.REVERSE, 10, .2);

        rd.lockFoundation("Lock");
        sleep(500);
        rd.scoreFoundationRed(10, 90);
        sleep(500);
        rd.moveDist(RobotDrive.Direction.REVERSE, 10, .75);
        rd.moveDist(RobotDrive.Direction.FORWARD,8,.75);
        rd.moveDist(RobotDrive.Direction.LEFT, 24, .75);
        rd.moveDist(RobotDrive.Direction.FORWARD,30,.75);

        /*
        //set gripper so it doesn't run into bridge
        hw.gripper.setPosition(.3);

        //lock foundation
        rd.moveDist(RobotDrive.Direction.REVERSE, 20, .5);
        rd.moveDist(RobotDrive.Direction.REVERSE, 10.5, .2);

        hw.f_servoRight.setPosition(1);
        hw.f_servoLeft.setPosition(.5);
        sleep(1500);

        
        //move and turn to be parallel to bridge
        
        rd.moveDist(RobotDrive.Direction.LEFT,17, .3);
        rd.proportionalTurn(90,1.5);
        rd.resetEncoders();

        //score foundation and unlock
        rd.moveDist(RobotDrive.Direction.RIGHT, 15, .5);
        rd.moveDist(RobotDrive.Direction.REVERSE, 18, .5);
        hw.f_servoRight.setPosition(.5);
        hw.f_servoLeft.setPosition(1);
        sleep(1500);

        //move to wall and park
        rd.moveDist(RobotDrive.Direction.RIGHT, 7, .5);
        rd.moveDist(RobotDrive.Direction.FORWARD, 37, .5);
        hw.f_servoLeft.setPosition(1);
        sleep(1500);
*/
    }

}
