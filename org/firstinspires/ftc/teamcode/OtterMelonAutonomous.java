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
import java.util.List;
import java.lang.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="SkyStoneAuto", group="Linear Opmode")
public class OtterMelonAutonomous extends LinearOpMode{
    private RobotHardware hw = new RobotHardware("OtterMelon", hardwareMap);
    private RobotDrive rd = new RobotDrive(hw); 
    private RobotSense rs= new RobotSense(hw);


    public void runOpMode(){

    //Vuforia, scan a picture and decide wheter you are on Loading side or Building side
        /*
    String whichSide= rs.scanBegin();

    //Loading zone: Robot starts in the loading zone 
    if(whichSide.equals("LOADING ZONE"){
        LoadingZone();
        */
    }


 public void LoadingZone(){
     /*
     //Start by scanning each stone to see if it is a skystone

        //if robot sees a skystone return true
        while(!rs.isSkystone()){
            //while the robot does not see a skyStone keep moving
            //rd.move(RobotDrive.Direction.RIGHT,8.0,  speed);
        }

        // robot now sees a skyskystone.
        // robotSense should give back the distance to which the robot needs to travel 
        // in an array [horizontal distance,forward distance]
        
        double[] distance= rs.locateSkystone();
        rd.move(RobotDrive.Direction.RIGHT,distance[0],double speed);
        rd.move(RobotDrive.Direction.FORWARD,distance[1],double speed);

        //drop the arm to drag the skystone 

        //go backward with the skystone
        rd.move(RobotDrive.Direction.REVERSE,double distance,double speed);
        rd.move(RobotDrive.Direction.LEFT,double distance,double speed); // direction needed to travel to deliver the stone

        rd.move(RobotDrive.Direction.RIGHT,double distance,double speed); // direction needed to travel to deliver the stone


        //move to the second skystone.
        //you are now infront of where the first skystone used to be,. you now where the second skystone is

        //the second skystone is 3 blocks over 
        rd.move(RobotDrive.Direction.Right,24,double speed);

        //drop the arm to drag the skystone 

        //go backward with the skystone
        rd.move(RobotDrive.Direction.REVERSE,double distance,double speed);
        rd.move(RobotDrive.Direction.LEFT,double distance,double speed); //direction needed to travel to deliver the stone


        //park
        rd.move(RobotDrive.Direction.RIGHT,double distance,double speed);
*/
 }
 public void BuildingZone(){
     /*
     //move the foundation
     rd.move(RobotDrive.Direction.RIGHT,double distance,double speed);

     //distance should be such that it takes the robot to be in front of the first stone.

     LoadingZone();
     */
 }
    

}