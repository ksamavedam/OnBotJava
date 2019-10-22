package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;

@Autonomous(name="SkyStoneAutoNew1", group="Linear Opmode")


public class AutoNew1 extends LinearOpMode {

    private Gyroscope imu;
    private DcMotor tlMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private DcMotor trMotor;
    private Blinker expansion_Hub_2;
    @Override
    public void runOpMode(){
        tlMotor=hardwareMap.get(DcMotor.class, "topLeft");
        blMotor=hardwareMap.get(DcMotor.class, "bottomLeft");
        brMotor=hardwareMap.get(DcMotor.class, "bottomRight");
        trMotor=hardwareMap.get(DcMotor.class, "topRight");
        tlMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);
        
        tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 

        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 
        tlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); 

        waitForStart();
                tlMotor.setTargetPosition(-1000);
                blMotor.setTargetPosition(1000);
                trMotor.setTargetPosition(-1000);
                brMotor.setTargetPosition(1000);

        while(opModeIsActive()){
        
        tlMotor.setPower(-.7); 
        blMotor.setPower(.7);
        brMotor.setPower(.7);
        trMotor.setPower(-.7);
        }
    }
}
