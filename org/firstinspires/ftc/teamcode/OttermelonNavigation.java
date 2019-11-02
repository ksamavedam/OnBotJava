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


public class OttermelonNavigation extends LinearOpMode{

    private BNO055IMU imu;
    private DcMotor tlMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private DcMotor trMotor;

    public void runOpMode(){

        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        Acceleration gravity;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        tlMotor=hardwareMap.get(DcMotor.class, "topLeft");
        blMotor=hardwareMap.get(DcMotor.class, "bottomLeft");
        brMotor=hardwareMap.get(DcMotor.class, "bottomRight");
        trMotor=hardwareMap.get(DcMotor.class, "topRight");
        
        tlMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
        // Prompt user to press start buton.
        telemetry.addData("IMU Example", "Press start to continue...");
        telemetry.update();
        
    }

    public void turnIMU(double targetAngle){
        double direction=1;
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if(targetAngle>180){
          direction=-1;
          targetAngle=targetAngle-360;
        }
        
        double delta = Math.abs(targetAngle-angles.firstAngle); 
        double power=.5;
        tlMotor.setPower(direction*power);
        blMotor.setPower(direction*power);
        brMotor.setPower(direction*-1*power);
        trMotor.setPower(direction*-1*power);
        while(delta >3.0){
          //if (1 == 1) break;
          angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
          delta = Math.abs(targetAngle-angles.firstAngle);
        }
        tlMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
        trMotor.setPower(0);
        
        
      }

      public static void proportionalTurnIMU(double targetAngle){

        double direction=1;
        Orientation angles;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if(targetAngle>180){
          direction=-1;
          targetAngle=targetAngle-360;
        }
        
        double delta = Math.abs(targetAngle-angles.firstAngle); 
        

        while(delta >3.0){
            
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            delta = Math.abs(targetAngle-angles.firstAngle);
            double power= delta*.5;
            tlMotor.setPower(direction*power);
            blMotor.setPower(direction*power);
            brMotor.setPower(direction*-1*power);
            trMotor.setPower(direction*-1*power);
          }

            tlMotor.setPower(0);
            blMotor.setPower(0);
            brMotor.setPower(0);
            trMotor.setPower(0);
        
      }
}