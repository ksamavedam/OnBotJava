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

public class RobotHandle extends LinearOpMode {
     

    @Override
    public void runOpMode(){
         RobotHardware rh=null;
         rh= new RobotHardware("OtterMelon", hardwareMap);
       
        public void moveIntake{
            if(gamepad.a){
                rh.leftMotor.setPower(0.5);.;
                rh.rightMotor.setPower(0.5);
            }

            if(gamepad.b){
                rh.intakeMotorLeft.setPower(0.5);.;
                rh.intakeMotorRight.setPower(0.5);;
            }
        }
            
    }

}