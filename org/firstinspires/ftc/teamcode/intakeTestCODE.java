package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import java.util.Arrays;
import java.lang.Math;

@TeleOp(name = "intakeTestCode", group = "TeleOpModes")

public class intakeTestCODE extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    
    @Override
    public void runOpMode(){
        
        leftMotor=hardwareMap.get(DcMotor.class, "leftIntake");
        rightMotor=hardwareMap.get(DcMotor.class, "rightIntake");
       
        waitForStart();
        
        while(opModeIsActive()){
            if(gamepad.a){
                leftMotor.setPower(0.3 );
                rightMotor.setPower(0.3);
            }
            if(gamepad.b){
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
         }
    }

}