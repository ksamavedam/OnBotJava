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

@Autonomous(name = "BlueLoading", group = "Linear Opmode")
public class OtterMelonAutonomousBlueLoading extends LinearOpMode {
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
        hw.f_servoLeft.setPosition(0.5);
        hw.f_servoRight.setPosition(0.5);
        rd.moveArm(hw.startPos());
        hw.gripper.setPosition(0);
        waitForStart();
        while (opModeIsActive()) {

            //.\bin\ftc_http_win.exe -ub
            LoadingZone();      
            break;
        }

        rs.shutdown();
    }

    


    public void playVuforia() {

        RobotSense.SSLocation ssl = rs.locateSkystone();
        // distance and angle are valid only if the skystone is detected.
        if (ssl.detected) {
            ssDetCount++;
            telemetry.addData("ITS A SKYSTONE, DetCount:", "%d ssNotDetCount:%d", ssDetCount, ssNotDetCount);
            telemetry.addData("Skystone DDist -HDist - Angle ", "%f  %f  %f", ssl.diagDistance, ssl.hzDistance, ssl.angle);
            telemetry.addData("actual distance", "%f", rs.getDistance());

        } else {
            ssNotDetCount++;
        }
        telemetry.update();
    }


    public void LoadingZone() {

        RobotSense.SSLocation ssl; 

        double position=1;
        double h_disp=0;
        
        //find skystone position
        ssl= rs.locateSkystone();
        hw.gripper.setPosition(0);
        if(!ssl.detected||ssl.angle>16){

            position=3;
            h_disp=10;
            rd.moveDist(RobotDrive.Direction.RIGHT, h_disp, .5);
        }
        else if((ssl.angle>-6&&ssl.angle<16)){

            position=2;
            h_disp=2;
            rd.moveDist(RobotDrive.Direction.RIGHT, h_disp, .5);
        }
        else{

            position=1;
            h_disp=8.75;
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
        hw.f_servoLeft.setPosition(1);
        hw.f_servoRight.setPosition(.5);

        rd.startIntake(.45);
        rd.moveDist(RobotDrive.Direction.FORWARD, 26, 1);
        rd.moveDist(RobotDrive.Direction.FORWARD, 21, .3);
        //rd.moveArm(.1);
        hw.gripper.setPosition(1);
        rd.startIntake(0);        
        rd.goDist(hw.distBack, 180, .5, 25, true);

        if(position == 1){

            rd.moveCorrect(90, 1, 0, 2.6);
        }
        else if(position == 2){

            rd.moveCorrect(90, 1, 0, 2.8);
        }
        else{

            rd.moveCorrect(90, 1, 0, 3.18);
        }
        rd.proportionalTurn(180,1.5);
        rd.goDist(hw.distBack, 180, .4, 1.7, true);
        rd.moveDist(RobotDrive.Direction.REVERSE, .3, .5);
        rd.lockFoundation("lock");

        autoStoneScore();
        rd.scoreFoundationRed(200, 270);
        rd.goDist(hw.distRight, 90, .5, 24, false);

        hw.gripper.setPosition(0);
        
        if(position == 1){

            rd.moveCorrect(0, 1, 270, 1.39);
        }
        else if(position == 2){

            rd.moveCorrect(0, 1, 270, 1.5);
        }
        else{

            rd.moveCorrect(0, 1, 270, 1.8);
        }

        rd.moveDist(RobotDrive.Direction.FORWARD, 13, 1);
        rd.startIntake(.45);
        rd.goDist(hw.distRight, 90, .5, 36, false);
        rd.moveDist(RobotDrive.Direction.FORWARD, 9, .5);
        rd.goDist(hw.distRight, 270, .5, 24, true);
        hw.gripper.setPosition(1);
        rd.startIntake(0);

        if(position == 1){

            rd.moveCorrect(180, 1, 270, 1.6);
        }
        else if(position == 2){

            rd.moveCorrect(180, 1, 270, 2.1);
        }
        else{

            rd.moveCorrect(180, 1, 270, 2.4);
        }
        rd.goDist(hw.distBack, 180, .5, 1.7, true);
        rd.moveDist(RobotDrive.Direction.RIGHT, 10, 1);

        autoStoneScore();
        rd.proportionalTurn(270,.5);
        rd.goDist(hw.distRight, 90, .5 , 24 , false);
        rd.moveDist(RobotDrive.Direction.FORWARD, 33, 1);




        /*
        rd.startIntake(.45);
        rd.moveDist(RobotDrive.Direction.FORWARD, 24, .5);
        rd.moveDist(RobotDrive.Direction.FORWARD, 23, .3);

        //deliever first skystone
        rd.moveDist(RobotDrive.Direction.REVERSE,22, .5);
        rd.startIntake(0);
        rd.proportionalTurn(270,1.5);
        hw.gripper.setPosition(.9);
        rd.resetEncoders();
        if(position==1){

            h_disp*=-1;
        }
        rd.moveDist(RobotDrive.Direction.REVERSE, 74+h_disp,.5);
        rd.moveArm(hw.highScore());
        rd.proportionalTurn(180, 1.5);
        rd.resetEncoders();
        rd.moveDist(RobotDrive.Direction.REVERSE, 5.75, .5);
        rd.moveDist(RobotDrive.Direction.REVERSE, 6.75, .25);
        rd.lockFoundation("Lock");
        hw.gripper.setPosition(.3);
        sleep(250);
        hw.gripper.setPosition(.9);
        sleep(250);
        rd.moveArm(hw.startPos());
        hw.gripper.setPosition(.8);
        rd.scoreFoundationRed(-170, -90);
        hw.gripper.setPosition(.3);
        sleep(700);

        /*
        rd.moveArm(hw.startPos());
        hw.gripper.setPosition(.8);
        //move and turn to be parallel to bridge
        
        rd.moveDist(RobotDrive.Direction.FORWARD, 3, .5);
        rd.moveDist(RobotDrive.Direction.LEFT,19, .5);
        rd.proportionalTurn(270,2);
        rd.resetEncoders();

        //score foundation and unlock
        rd.moveDist(RobotDrive.Direction.RIGHT, 50, .5);
        rd.moveDist(RobotDrive.Direction.LEFT, 17, .5);
        rd.moveDist(RobotDrive.Direction.REVERSE, 20, .5);
        hw.f_servoRight.setPosition(.5);
        hw.f_servoLeft.setPosition(1);
        sleep(500);
        hw.f_servoLeft.setPosition(1);

        //move to wall and park
        rd.moveDist(RobotDrive.Direction.LEFT, 3, .5);
        rd.moveDist(RobotDrive.Direction.FORWARD, 42, .75);
        
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
