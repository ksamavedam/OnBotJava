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

@Autonomous(name = "BlueBuilding", group = "Linear Opmode")
public class OtterMelonAutonomousBlueBuilding extends LinearOpMode {
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
        hw.f_servoRight.setPosition(.5);
        waitForStart();
        while (opModeIsActive()) {

            BuildingZone();
            break;
        }

        rs.shutdown();
    }

    public void LoadingZone() {

        RobotSense.SSLocation ssl; 

        double position=1;
        double h_disp=0;
        //get: rs. position; 
        //rd.moveDist(RobotDrive.Direction.LEFT, 10, .5);
        //wobble
        rd.moveDist(RobotDrive.Direction.FORWARD, .5, .3);
        rd.moveDist(RobotDrive.Direction.REVERSE, .5, .3);

        //find skystone position
        ssl= rs.locateSkystone();
        if(!ssl.detected){

            position=2;
            h_disp=10;
            rd.moveDist(RobotDrive.Direction.RIGHT, h_disp, .5);
        }
        else if((ssl.angle>-5)){

            position=2;
            h_disp=2;
            rd.moveDist(RobotDrive.Direction.RIGHT, h_disp, .5);
        }
        /*else if(ssl.angle>13){

            h_disp=12.5;
            position=3;
            //rd.moveDist(RobotDrive.Direction.RIGHT, h_disp, .5);
        }*/
        else{

            position=1;
            h_disp=7.5;
            rd.moveDist(RobotDrive.Direction.LEFT, h_disp, .5);
        }
        
        if(ssl.detected){
        telemetry.addData("position", position);
        //telemetry.addData("Skystone DDist -HDist - Angle ", "%f  %f  %f", ssl.diagDistance, ssl.hzDistance, ssl.angle);

        telemetry.addData("angle", ssl.angle);
        telemetry.update();
        }
        else{
            telemetry.addData("NOT DETECTED", position);
            telemetry.update();
    
        }

        //grab first skystone
        rd.startIntake(-.45);
        rd.moveDist(RobotDrive.Direction.FORWARD, 44.7, .3);
        

        //deliever first skystone
        rd.moveDist(RobotDrive.Direction.REVERSE,25, 1);
        rd.proportionalTurn(90, 1.5);
        rd.resetEncoders();
        if(position==1){

            h_disp*=-1;
        }
        rd.moveDist(RobotDrive.Direction.FORWARD, 60+h_disp, 1);
        rd.startIntake(.6);
        rd.proportionalTurn(90, 1);

        //go back to loading zone
        rd.moveDist(RobotDrive.Direction.REVERSE,60+h_disp+24,.8);
        
       
        
        //get second skystone
        rd.startIntake(-.45);
        rd.proportionalTurn(1  , 1.5);
        rd.resetEncoders();
        rd.moveDist(RobotDrive.Direction.FORWARD, 25, .3);

        //deliver second skystone
        rd.moveDist(RobotDrive.Direction.REVERSE,35, .7);
        rd.proportionalTurn(90,1.5);
        rd.resetEncoders();
        rd.moveDist(RobotDrive.Direction.FORWARD, 75+h_disp, 1);
        rd.startIntake(.6);

        //park
        rd.moveDist(RobotDrive.Direction.REVERSE, 10, .5);

        

        
    }

    
    public void BuildingZone() {

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
        rd.moveDist(RobotDrive.Direction.REVERSE, 16, .5);
        hw.f_servoRight.setPosition(.5);
        hw.f_servoLeft.setPosition(1);
        sleep(1500);

        //move to wall and park
        rd.moveDist(RobotDrive.Direction.RIGHT, 40, .5);
        rd.moveDist(RobotDrive.Direction.FORWARD, 37, .5);
        

    }

}
