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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Arrays;
import java.lang.Math;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

@TeleOp(name = "TestingTele", group = "TeleOpModes")

public class TestingTele extends LinearOpMode {
    private BNO055IMU imu;

    public ElapsedTime mRunTime = new ElapsedTime();

    public Servo gripper;
    public Servo armRight;
    public Servo armLeft;
    public Servo level;
    RobotHardware rh = null;
    RobotDrive rd = null;

    @Override
    public void runOpMode() {
        gripper=hardwareMap.get(Servo.class, "gripper");
        armLeft=hardwareMap.get(Servo.class, "armLeft");
        armRight=hardwareMap.get(Servo.class, "armRight");
        level=hardwareMap.get(Servo.class, "level");
        rh = new RobotHardware("OtterMelon", hardwareMap);
        rd = new RobotDrive(rh);
    
        waitForStart();
        double s1Pos=.3;
        while (opModeIsActive()) {
           
            double angle = 0;
            //Finding orientation of the right stick
            if (gamepad1.right_stick_x > 0 && gamepad1.right_stick_y <= 0) {
                angle = Math.atan(-gamepad1.right_stick_y / gamepad1.right_stick_x) + 3 * Math.PI / 2;
            } else if (gamepad1.right_stick_x >= 0 && gamepad1.right_stick_y > 0) {
                angle = Math.atan(gamepad1.right_stick_x / gamepad1.right_stick_y) + Math.PI;
            } else if (gamepad1.right_stick_x < 0 && gamepad1.right_stick_y >= 0) {
                angle = Math.atan(gamepad1.right_stick_y / -gamepad1.right_stick_x) + Math.PI / 2;
            } else if (gamepad1.right_stick_x <= 0 && gamepad1.right_stick_y < 0) {
                angle = Math.atan(gamepad1.right_stick_x / gamepad1.right_stick_y);
            } else {
                angle = 0;
            }

            double scale = Math.sqrt(((gamepad1.right_stick_y) * (gamepad1.right_stick_y))
                    + ((gamepad1.right_stick_x) * (gamepad1.right_stick_x))) ;

            //Rotating one
            double turnScale = gamepad1.left_stick_x*.75;

            rd.moveTeleop(angle, scale, turnScale);
            telemetry.addData("Left: cm", "%.2f cm", rh.distLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("Back: cm", "%.2f cm", rh.distBack.getDistance(DistanceUnit.CM));
            telemetry.update();        
        }
    }
}