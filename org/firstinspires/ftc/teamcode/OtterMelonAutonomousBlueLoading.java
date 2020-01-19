//.\bin\ftc_http_win.exe -ub
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

        //Start the servos in the correct starting position
        /*hw.armRight.setPosition(1-.3);
        hw.armLeft.setPosition(.3);
        hw.level.setPosition(.3+.05);*/
        hw.f_servoLeft.setPosition(0.5);
        hw.f_servoRight.setPosition(0.5);
        double startPos=.04;
        hw.armLeft.setPosition(startPos);
        hw.armRight.setPosition(1-startPos);
        hw.level.setPosition(startPos+.05);
        hw.gripper.setPosition(0);
        waitForStart();
        while (opModeIsActive()) {

            
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
        //get: rs. position; 
        //rd.moveDist(RobotDrive.Direction.LEFT, 10, .5);
        //wobble
        /*rd.moveDist(RobotDrive.Direction.FORWARD, .5, .3);
        rd.moveDist(RobotDrive.Direction.REVERSE, .5, .3);
        hw.gripper.setPosition(0);*/

        //Locate skystone's position
        //Align the robot to be in front of the skystone
        ssl= rs.locateSkystone();
        hw.gripper.setPosition(.3);
        if(!ssl.detected||ssl.angle>16){

            position=3;
            h_disp=10;
            rd.moveDist(RobotDrive.Direction.RIGHT, h_disp, .5);
        }
        else if(ssl.angle>13){

            h_disp=12.5;
            position=3;
            rd.moveDist(RobotDrive.Direction.RIGHT, h_disp, .5);
        }
        else if((ssl.angle>-5&&ssl.angle<16)){

            position=2;
            h_disp=2;
            rd.moveDist(RobotDrive.Direction.RIGHT, h_disp, .5);
        }
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

        
        //Bring up the foundation servos
        hw.f_servoLeft.setPosition(1);
        hw.f_servoRight.setPosition(.5);

        //Move forward to grab the skystone
        rd.startIntake(.45);
        rd.moveDist(RobotDrive.Direction.FORWARD, 24, .5);
        rd.moveDist(RobotDrive.Direction.FORWARD, 23, .3);

        //Deliver the skystone to the building zone
        rd.moveDist(RobotDrive.Direction.REVERSE,19, .5);
        rd.proportionalTurn(270,1.5);
        rd.resetEncoders();
        if(position==1){

            h_disp*=-1;
        }
        rd.moveDist(RobotDrive.Direction.REVERSE, 74+h_disp,.5);

        //Stop the intake
        rd.startIntake(0);
        double s1Pos=0;

        //Move the servos into the robot to grab skystone
        hw.armRight.setPosition(1-s1Pos);
        hw.armLeft.setPosition(s1Pos);
        hw.level.setPosition(s1Pos+.08);
        sleep(500);
        hw.gripper.setPosition(.8);
        sleep(1000);
        s1Pos=.5;
        
        //Move servos into placing position
        hw.armRight.setPosition(1-s1Pos);
        hw.armLeft.setPosition(s1Pos);
        hw.level.setPosition(s1Pos+.08);
        rd.proportionalTurn(180, 1.5);
        rd.resetEncoders();
        //Align to the foundation
        rd.moveDist(RobotDrive.Direction.REVERSE, 4.75, .5);
        rd.moveDist(RobotDrive.Direction.REVERSE, 6.75, .25);

        hw.f_servoRight.setPosition(1);
        hw.f_servoLeft.setPosition(.5);
        hw.armRight.setPosition(1-s1Pos);
        hw.armLeft.setPosition(s1Pos);
        hw.level.setPosition(s1Pos+.08);
        sleep(1500);

        //Release the skystone on the foundation
        hw.gripper.setPosition(.3);
        sleep(500);
        s1Pos=.03;
        sleep(500);

        //Move the arm back inside the robot
        hw.armRight.setPosition(1-s1Pos);
        hw.armLeft.setPosition(s1Pos);
        sleep(500);
        hw.gripper.setPosition(.8);
        hw.level.setPosition(s1Pos+.08);

        //Algin robot to be parallel to the bridge
        rd.moveDist(RobotDrive.Direction.LEFT,19, .5);
        rd.proportionalTurn(270,2);
        rd.resetEncoders();

        //Move the foundation into the building zone
        rd.moveDist(RobotDrive.Direction.RIGHT, 50, .5);
        rd.moveDist(RobotDrive.Direction.LEFT, 15, .5);
        rd.proportionalTurn(270,1.5);
        rd.moveDist(RobotDrive.Direction.REVERSE, 20, .5);

        //Unlatch from the founation
        hw.f_servoRight.setPosition(.5);
        hw.f_servoLeft.setPosition(1);
        sleep(1500);
        hw.f_servoLeft.setPosition(1);

        //Move under the bridge and park
        rd.moveDist(RobotDrive.Direction.LEFT, 3, .5);
        rd.moveDist(RobotDrive.Direction.FORWARD, 42, .5);
        

        
        

    }

    

}
