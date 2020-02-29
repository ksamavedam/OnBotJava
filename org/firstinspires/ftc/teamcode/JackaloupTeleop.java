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

@TeleOp(name = "JackaloupTeleop", group = "TeleOpModes")

public class JackaloupTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware rh = null;
        RobotDrive rd = null;
        rh = new RobotHardware("Jackaloup", hardwareMap);
        rd = new RobotDrive(rh);
        waitForStart();
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
            telemetry.addData("Angle: ", Math.toDegrees(angle));

            //Desired power of moving
            double scale = Math.sqrt(((gamepad1.right_stick_y) * (gamepad1.right_stick_y))
                    + ((gamepad1.right_stick_x) * (gamepad1.right_stick_x))) * .5;

            //Rotating one
            double turnScale = -gamepad1.left_stick_x*.75;
            telemetry.addData("Scale", scale);
            telemetry.addData("turnScale", turnScale);
            telemetry.update();

            if(gamepad1.a){

                rh.tlMotor.setPower(.5);
            }
            else if(gamepad1.b){

                rh.blMotor.setPower(.5);
            }
            else if(gamepad1.x){

                rh.brMotor.setPower(.5);
            }
            else if(gamepad1.y){

                rh.trMotor.setPower(.5);
            }
            else {

                rd.moveTeleop(angle, scale, turnScale);
            }
        }
    }
}
       