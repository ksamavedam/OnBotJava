package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Arrays;
import java.lang.Math;

@TeleOp(name = "FrameofReferenceDrive", group = "TeleOpModes")

public class FrameofReferenceDrive extends LinearOpMode {
    private BNO055IMU imu;
    private DcMotor tlMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private DcMotor trMotor;
    private Blinker expansion_Hub_2;
    private Servo grabber;
    private DcMotor grabberArm;
    private Servo foundationServo;
        @Override
    public void runOpMode(){
        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        
        tlMotor=hardwareMap.get(DcMotor.class, "topLeft");
        blMotor=hardwareMap.get(DcMotor.class, "bottomLeft");
        brMotor=hardwareMap.get(DcMotor.class, "bottomRight");
        trMotor=hardwareMap.get(DcMotor.class, "topRight");
        grabberArm=hardwareMap.get(DcMotor.class, "grabber arm");
        grabber=hardwareMap.get(Servo.class, "grabber");
        foundationServo=hardwareMap.get(Servo.class, "foundationServo");
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
        
        telemetry.addData("IMU: ", "done");
        telemetry.update();
        
        
        waitForStart();
        
        
        while(opModeIsActive()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double offset =  -1*Math.toRadians(angles.firstAngle);
            double angle= 0;
            if (gamepad1.right_stick_x > 0 && gamepad1.right_stick_y <= 0) {
                angle = Math.atan(-gamepad1.right_stick_y/gamepad1.right_stick_x)+3*Math.PI/2;
            }
            else if (gamepad1.right_stick_x >= 0 && gamepad1.right_stick_y > 0) {
                angle = Math.atan(gamepad1.right_stick_x/gamepad1.right_stick_y)+Math.PI;
            }
            else if (gamepad1.right_stick_x < 0 && gamepad1.right_stick_y >= 0) {
                angle = Math.atan(gamepad1.right_stick_y/-gamepad1.right_stick_x)+Math.PI/2;
            }
            else if (gamepad1.right_stick_x <= 0 && gamepad1.right_stick_y < 0) {
                angle = Math.atan(gamepad1.right_stick_x/gamepad1.right_stick_y);
            }
            else {
                angle = 0;
            }
            angle+=offset;
            telemetry.addData("offset", offset);
            telemetry.addData("Angle: ", angle);
            telemetry.update();
            double scale= Math.sqrt(((gamepad1.right_stick_y)*(gamepad1.right_stick_y))+((gamepad1.right_stick_x)*(gamepad1.right_stick_x)));
            double turnScale= gamepad1.left_stick_x;
            double scale2 = .75*10;
            double[] powers=OttermelonMotion.move(angle,scale,turnScale);
            tlMotor.setPower(powers[0]*scale2);
            blMotor.setPower(powers[1]*scale2);
            brMotor.setPower(powers[2]*scale2);
            trMotor.setPower(powers[3]*scale2);
            
            
            /*if(gamepad2.x){
                grabberArm.setTargetPosition(-220);
                grabberArm.setPower(.2);
            }
            else if(gamepad2.y){
                grabberArm.setTargetPosition(-80);
                grabberArm.setPower(.2);
                
            }*/
            
            //Foundation Mover
            if(gamepad2.left_bumper){
                foundationServo.setPosition(0);
            }
            else if(gamepad2.right_bumper){
                foundationServo.setPosition(1);
            }
            
            //Grabber Arm
            if(gamepad2.left_trigger>0){
                grabberArm.setPower(.3);
            }
            else if(gamepad2.right_trigger>0){
                grabberArm.setPower(-.3);
            }
            else{
                grabberArm.setPower(0);
            }
            
            //Grabber Servo
            if (gamepad2.a){
                grabber.setPosition(1);
            }
            else if (gamepad2.b){
                grabber.setPosition(0);
            }
            
            /*telemetry.addData("Angle: ", Math.toDegrees(angle));
            telemetry.addData("Turn Scale: ", turnScale);
            telemetry.addData("Top Left: ", powers[0]);
            telemetry.addData("Bottom Left: ", powers[1]);
            telemetry.addData("Bottom Right: ", powers[2]);
            telemetry.addData("Top Right: ", powers[3]);
            telemetry.update();/*
        }
        /*if(gamepad1.left_stick_x>0){
                tlMotor.setPower(gamepad1.left_stick_x);
                blMotor.setPower(gamepad1.left_stick_x);
                brMotor.setPower(-gamepad1.left_stick_x);
                trMotor.setPower(-gamepad1.left_stick_x);
            }*/

    }
     
    


}
}
