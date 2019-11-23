  
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
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

@TeleOp(name = "OTttermelonTeleop", group = "TeleOpModes")

public class OttermelonTeleop extends LinearOpMode {
    private BNO055IMU imu;
    
    public ElapsedTime mRunTime= new ElapsedTime();
    
    @Override
    public void runOpMode(){
        /*BNO055IMU.Parameters imuParameters;
        Orientation angles;*/
        
        /*tlMotor=hardwareMap.get(DcMotor.class, "topLeft");
        blMotor=hardwareMap.get(DcMotor.class, "bottomLeft");
        brMotor=hardwareMap.get(DcMotor.class, "bottomRight");
        trMotor=hardwareMap.get(DcMotor.class, "topRight");
        
        imu = hardwareMap.get(BNO055IMU.class, "imu");
       
        //tlMotor.setDirection(DcMotor.Direction.REVERSE);
        brMotor.setDirection(DcMotor.Direction.REVERSE);
        trMotor.setDirection(DcMotor.Direction.REVERSE);
        //blMotor.setDirection(DcMotor.Direction.REVERSE); 
        
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
        */
         RobotHardware rh=null;
         RobotDrive rd=null;
        rh= new RobotHardware("OtterMelon", hardwareMap);
        rd=new RobotDrive(rh);
        waitForStart();
        while(opModeIsActive()){

           Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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
            telemetry.addData("Angle: ", Math.toDegrees(angle));
            double scale= Math.sqrt(((gamepad1.right_stick_y)*(gamepad1.right_stick_y))+((gamepad1.right_stick_x)*(gamepad1.right_stick_x)))*.75;
            double turnScale= gamepad1.left_stick_x;
            telemetry.addData("Scale", scale);
            telemetry.addData("turnScale", turnScale);
            telemetry.update();
            rd.moveTeleop(angle,scale,turnScale);
            if(gamepad1.a){

                rh.tlMotor.setPower(.5);
            }
            else if(gamepad1.b){

                rh.blMotor.setPower(.5);
            }
            else if(gamepad1.x){

                rh.brMotor.setPower(.4);
            }
            else if(gamepad1.y){

                rh.trMotor.setPower(.4);
            }
            
    }
     
    


}



}


