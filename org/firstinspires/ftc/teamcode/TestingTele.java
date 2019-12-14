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

@TeleOp(name = "TestingTele", group = "TeleOpModes")

public class TestingTele extends LinearOpMode {
    private BNO055IMU imu;

    public ElapsedTime mRunTime = new ElapsedTime();

    public Servo gripper;
    public Servo armRight;
    public Servo armLeft;
    public Servo level;
    @Override
    public void runOpMode() {
        gripper=hardwareMap.get(Servo.class, "gripper");
        armLeft=hardwareMap.get(Servo.class, "armLeft");
        armRight=hardwareMap.get(Servo.class, "armRight");
        level=hardwareMap.get(Servo.class, "level");
    
        waitForStart();
        double s1Pos=.3;
        while (opModeIsActive()) {
           
            if (gamepad1.a) {

                gripper.setPosition(0);
                s1Pos=.4;
            } else if (gamepad1.b) {
                telemetry.addData("HI", "HI");
                gripper.setPosition(.5);
                s1Pos=.2;
            } 
            else if (gamepad1.x) {
                gripper.setPosition(1);
                s1Pos=.3;
            } else if (gamepad1.y) {

                s1Pos=.5;
            }
            
            
            /*armRight.setPosition(1-s1Pos);
            armLeft.setPosition(s1Pos);
            level.setPosition(s1Pos+.08);*/
        }
    }
}