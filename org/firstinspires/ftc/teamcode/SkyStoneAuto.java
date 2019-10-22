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

public class SkyStoneAuto extends LinearOpMode{
    private BNO055IMU imu;
    private DcMotor tlMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private DcMotor trMotor;
    private DcMotor grabberMotor;
    private Servo foundationServo;
    private Servo grabber;
    private Blinker expansion_Hub_2;
    @Override
    public void runOpMode(){
        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        
        tlMotor=hardwareMap.get(DcMotor.class, "topLeft");
        blMotor=hardwareMap.get(DcMotor.class, "bottomLeft");
        brMotor=hardwareMap.get(DcMotor.class, "bottomRight");
        trMotor=hardwareMap.get(DcMotor.class, "topRight");
        grabberMotor=hardwareMap.get(DcMotor.class, "grabber arm");
        
        foundationServo=hardwareMap.get(Servo.class,"foundationServo");
        grabber=hardwareMap.get(Servo.class,"grabber" );
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        
        tlMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);
        
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
        
        
        tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        tlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        grabberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        foundationServo.setPosition(0);
        grabber.setPosition(1);
        
        
        boolean flag= false;
        telemetry.addData("IMU: ", "done");
        telemetry.update();
        
        waitForStart();
        while(opModeIsActive()){
            
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            
            //move to to foundation
            moveEnc(180,.2,0,5);
            moveEnc(270,.2,0,270);
            moveEnc(180,.2,0,37);
            
            //lock foundation and resposition
            foundationServo.setPosition(1);
            moveEnc(0,.05,0,1);
            moveEnc(0,.2,0,40);
            
            //move to quarry
            foundationServo.setPosition(0);
            moveEnc(90,.5,0,40);
            moveEnc(180,.5,0,20);
            moveEnc(270,.5,0,1000);
            moveEnc(90,.5,0,6);
            moveEnc(0,.2,0,7);
            moveEnc(90,.5,0,100);
            moveEnc(180,0,.5,100);
            grabberMotor.setTargetPosition(-220);
            grabberMotor.setPower(.5);
            grabber.setPosition(0);
            moveEnc(0,.05,0,1);  
            
            //grab stone
            moveEnc(0,.35,0,24);
            moveEnc(0,.05,0,1);
            grabber.setPosition(1);
            moveEnc(180,.05,0,1);
            moveEnc(180,.5,0,10);
            
            //Deliver stone to build zone
            grabberMotor.setTargetPosition(-100);
            grabberMotor.setPower(.5);
            moveEnc(90,.5,0,100);
            grabber.setPosition(0);
            moveEnc(270,.05,0,500);
            moveEnc(180,.05,0,1);
            
    
            tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addData("arm motor ticks", grabberMotor.getCurrentPosition());
            telemetry.update();
        }
        
    }
    public void moveEnc(double angle, double scale, double turnScale, double distEnc){
        //Converts input angle to radians
        double r_angle=Math.toRadians(angle);
        
        //Constants for moving in each direction
        double ticksToInchV=32.0;
        double ticksToInchH=37.0;
        double ticksToInchR=15.0;
        double ticksToInchD=49.0;
        
        //getting powers to go in desired angle
        double[] powers= OttermelonMotion.move(r_angle, scale, turnScale);
        double encoders;
        
        // In this auto, we don't turn and move straight at the same time
        //Below block is for moving over plane
        if(scale != 0) {
            
            //next block checks which angle we are going at, sets the encoder counts by mulitplying by that constant, 
            // then sets the motors to run to that encoder position using run to position
            if(angle==0){
                encoders = distEnc*ticksToInchV;
                tlMotor.setTargetPosition((int)encoders);
                blMotor.setTargetPosition((int)encoders);
                brMotor.setTargetPosition((int)encoders);
                trMotor.setTargetPosition((int)encoders);
            }
            else if(angle==180){
                encoders = distEnc*ticksToInchV;
                tlMotor.setTargetPosition(-(int)encoders);
                blMotor.setTargetPosition(-(int)encoders);
                brMotor.setTargetPosition(-(int)encoders);
                trMotor.setTargetPosition(-(int)encoders);
            }
            else if(angle==90){
                encoders=distEnc+ticksToInchH;
                tlMotor.setTargetPosition((int)encoders*-1);
                blMotor.setTargetPosition((int)encoders);
                brMotor.setTargetPosition((int)encoders*-1);
                trMotor.setTargetPosition((int)encoders);
            }
            else{
                encoders = distEnc*ticksToInchH;
                tlMotor.setTargetPosition((int)encoders);
                blMotor.setTargetPosition((int)encoders*-1);
                brMotor.setTargetPosition((int)encoders);
                trMotor.setTargetPosition((int)encoders*-1);
            }
        }
        else {
            //does the same thing as aboove excpect it does it for turning
            encoders = distEnc*ticksToInchR;
            if(turnScale < 0) {
                tlMotor.setTargetPosition((int)encoders);
                blMotor.setTargetPosition((int)encoders);
                brMotor.setTargetPosition(-1 * (int)encoders);
                trMotor.setTargetPosition(-1 * (int)encoders);
            }
            else {
                tlMotor.setTargetPosition(-1 * (int)encoders);
                blMotor.setTargetPosition(-1 * (int)encoders);
                brMotor.setTargetPosition((int)encoders);
                trMotor.setTargetPosition((int)encoders);
            }

        }
        
        //gives motor their powers
        tlMotor.setPower(powers[0]);
        blMotor.setPower(powers[1]);
        brMotor.setPower(powers[2]);
        trMotor.setPower(powers[3]);
        telemetry.addData("Test",0);
        telemetry.addData("TL", powers[0]);
        telemetry.addData("BL", powers[1]);
        telemetry.addData("BR", powers[2]);
        telemetry.addData("TR", powers[3]);
        telemetry.update();
        
        //does not give next command until motors have reached their position
        while(opModeIsActive() && tlMotor.isBusy()){
            
        }
        resetEncoders();
    }
    
    //resets after each move
    public void resetEncoders(){
        tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        tlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
   
    //returns powers of each motor to move at a certain angle
    public double[] move(double angle, double scale, double turnScale){
        double topLeft=0;
        double bottomLeft=0;
        double bottomRight=0;
        double topRight= 0;
        double[] powers= new double[4];
        
        if(scale!=0.0){
            topLeft=(Math.cos(angle)/Math.sqrt(2)) + -1*(Math.sin(angle))+turnScale*1;
            bottomLeft=(Math.cos(angle)/Math.sqrt(2)) + (Math.sin(angle))+turnScale*1;
            bottomRight=(Math.cos(angle)/Math.sqrt(2)) + -1*(Math.sin(angle))+turnScale*-1;
            topRight= (Math.cos(angle)/Math.sqrt(2)) + (Math.sin(angle))+turnScale*-1;
            powers[0]=topLeft;
            powers[1]=bottomLeft;
            powers[2]=bottomRight;
            powers[3]=topRight;
            
            for(int i=0; i<powers.length; i++){
                powers[i]=Math.abs(powers[i]);
            }
            Arrays.sort(powers);
            topLeft=topLeft/powers[3];
            bottomLeft=bottomLeft/powers[3];
            bottomRight=bottomRight/powers[3];
            topRight=topRight/powers[3];
            topLeft*=scale;
            bottomLeft*=scale;
            bottomRight*=scale;
            topRight*=scale;   
            powers[0]=topLeft;
            powers[1]=bottomLeft;
            powers[2]=bottomRight;
            powers[3]=topRight;
        }
        else {
            powers[0] = 1*turnScale;
            powers[1] = 1*turnScale;
            powers[2] = -1*turnScale;
            powers[3] = -1*turnScale;
            
        }
        return powers;
    }
    
    public  void spinIMU(double targetAngle,Orientation angles, double power){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle=angles.firstAngle;
        if(targetAngle>180){
            targetAngle=-1*(360-targetAngle);
        }
        while(currentAngle!=targetAngle){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentAngle= angles.firstAngle;
        }
    }

    
}

