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

@Autonomous(name = "RedLoading", group = "Linear Opmode")
public class OtterMelonAutonomousRedLoading extends LinearOpMode {
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
        // Initialize hardware, drive and sense
        hw = new RobotHardware(robotName, hardwareMap);
        rd = new RobotDrive(hw);
        rs=new RobotSense(hw, telemetry);

        telemetry.addData("Ready! ", "Go Flamangos!"); 
        telemetry.update();

        //set servo start positions
        rd.lockFoundation("unlock");
        rd.moveArm(hw.startPos());
        
        waitForStart();
        while (opModeIsActive()) {

            //.\bin\ftc_http_win.exe -ub
            LoadingZone();      
            break;
        }

        rs.shutdown();
    }

    public void LoadingZone() {

        RobotSense.SSLocation ssl; 

        double position=0 ;
        double h_disp=0;

        hw.gripper.setPosition(0);

        //find skystone position
        ssl= rs.locateSkystone();
        if(!ssl.detected){

            position=3;
            h_disp=12;
            rd.moveDist(RobotDrive.Direction.LEFT, h_disp, .5);
        }
        else if((ssl.angle>-6)){

            position=1;
            h_disp=8;
            rd.moveDist(RobotDrive.Direction.RIGHT, h_disp, .5);
        }
        else {

            position=2;
            h_disp=3;
            rd.moveDist(RobotDrive.Direction.LEFT, h_disp, .5);
        }
        
        if(ssl.detected){
        telemetry.addData("position", position);
        telemetry.addData("angle", ssl.angle);
        telemetry.update();
        }
        else{
            telemetry.addData("NOT DETECTED", position);
            telemetry.update();
    
        }

       
       //grab first skystone
        rd.startIntake(.45);
        rd.moveDist(RobotDrive.Direction.FORWARD, 26, 1);
        rd.moveDist(RobotDrive.Direction.FORWARD, 21, .3);
        //rd.moveArm(.1);
        hw.gripper.setPosition(1);
        rd.startIntake(0);        
        rd.goDist(hw.distBack, 180, .5, 25, true);
        
        if(position==1){

            h_disp*=-1;
        }
        
        if(position == 1){

            rd.moveCorrect(270, 1, 0, 2.4);
        }
        else if(position == 2){

            rd.moveCorrect(270, 1, 0, 2.6);
        }
        else{

            rd.moveCorrect(270, 1, 0, 3);
        }
        rd.proportionalTurn(180,1.5);
        rd.goDist(hw.distBack, 180, .4, 1.7, true);
        rd.moveDist(RobotDrive.Direction.REVERSE, .3, .5);
        rd.lockFoundation("lock");

        autoStoneScore();

        
        sleep(500);
        rd.scoreFoundationRed(160, 90);
        rd.goDist(hw.distLeft, 270, .5, 24, false);

        hw.gripper.setPosition(0);
        
        if(position == 1){

            rd.moveCorrect(0, 1, 90, 1.5);
        }
        else if(position == 2){

            rd.moveCorrect(0, 1, 90, 1.6);
        }
        else{

            rd.moveCorrect(0, 1, 90, 1.9);
        }

        rd.moveDist(RobotDrive.Direction.FORWARD, 13, 1);
        rd.startIntake(.6);
        rd.goDist(hw.distLeft, 270, .5, 39, false);
        rd.moveDist(RobotDrive.Direction.FORWARD, 8, .75);
        rd.goDist(hw.distLeft, 90, .5, 24, true);
        hw.gripper.setPosition(1);
        rd.startIntake(0);
        if(position == 1){

            rd.moveCorrect(180, 1, 90, 1.5);
        }
        else if(position == 2){

            rd.moveCorrect(180, 1, 90, 2);
        }
        else{

            rd.moveCorrect(180, 1, 90, 2.3);
        }
        rd.goDist(hw.distBack, 180, .5, 1.7, true);
        rd.moveDist(RobotDrive.Direction.LEFT, 10, 1);;

        autoStoneScore();
        rd.goDist(hw.distLeft, 270, .5 , 23, false);
        rd.moveDist(RobotDrive.Direction.FORWARD, 33, 1);
        

/*
        //deliver first skystone
        rd.moveDist(RobotDrive.Direction.REVERSE, 66+h_disp, 1);
        rd.moveArm(hw.highScore());
        rd.proportionalTurn(180,1.5);

        //place first skystone
        rd.moveDist(RobotDrive.Direction.REVERSE, 6, .5);
        hw.gripper.setPosition(.3);
        sleep(250);
        hw.gripper.setPosition(.9);
        sleep(250);
        rd.moveArm(hw.startPos());
        hw.gripper.setPosition(.8);

        //get second skystone
        rd.moveDist(RobotDrive.Direction.FORWARD, 6, 1);
        rd.proportionalTurn(90, 1.5);
        hw.gripper.setPosition(.3);
        rd.moveDist(RobotDrive.Direction.FORWARD, 68+h_disp + 14, 1);
        rd.startIntake(.6);
        rd.moveDist(RobotDrive.Direction.RIGHT, 20.5, .6);
        rd.moveDist(RobotDrive.Direction.FORWARD, 7, 1);

        //delviver second skystone
        rd.moveDist(RobotDrive.Direction.REVERSE, 7, 1);
        rd.moveDist(RobotDrive.Direction.LEFT, 22.5, 1);
        rd.moveArm(.1);
        hw.gripper.setPosition(1);
        rd.proportionalTurn(90);
        rd.startIntake(0);
        rd.moveDist(RobotDrive.Direction.REVERSE, 68+h_disp + 17, 1);

        //place second skystone
        rd.moveArm(hw.highScore());
        rd.proportionalTurn(180,1);
        rd.moveDist(RobotDrive.Direction.REVERSE, 4, .75);
        rd.moveDist(RobotDrive.Direction.REVERSE, 8, .2);
        hw.gripper.setPosition(.3);
        
        //score foundation
        rd.lockFoundation("Lock");
        sleep(500);
        rd.scoreFoundationRed(170,90);
        /*
        sleep(500);
        hw.gripper.setPosition(.9);
        sleep(500);
        rd.moveArm(hw.startPos());
        hw.gripper.setPosition(.8);
        //rd.scoreFoundationRed();



        /*
        //score foundation 
        rd.moveDist(RobotDrive.Direction.RIGHT,20, .3);
        rd.proportionalTurn(90,1.5);
        rd.moveDist(RobotDrive.Direction.LEFT, 15.5, .5);
        rd.moveDist(RobotDrive.Direction.REVERSE, 16, .5);
        hw.f_servoRight.setPosition(.5);
        hw.f_servoLeft.setPosition(1);
        sleep(500);

        //move to wall and park
        rd.moveDist(RobotDrive.Direction.LEFT, 14, .5);
        rd.moveDist(RobotDrive.Direction.FORWARD, 37, .75);
        


        
        
*/
    }

    public void autoStoneScore(){

        rd.moveArm(hw.highScore());
        sleep(500);
        hw.gripper.setPosition(.45);
        sleep(500);
        hw.gripper.setPosition(0);
        sleep(500);
        rd.moveArm(hw.startPos());
    }

    

}
