
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

@TeleOp(name = "OtttermelonTeleop", group = "TeleOpModes")

public class OttermelonTeleop extends LinearOpMode {
    private BNO055IMU imu;

    public ElapsedTime mRunTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        RobotHardware rh = null;
        RobotDrive rd = null;
        rh = new RobotHardware("OtterMelon", hardwareMap);
        rd = new RobotDrive(rh);
        waitForStart();
        while (opModeIsActive()) {
            Orientation angles= rh.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double offset = -1 * Math.toRadians(angles.firstAngle);
            double angle = 0;
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

           // angle += offset;
            telemetry.addData("offset", offset);
            telemetry.addData("Angle: ", Math.toDegrees(angle));
            double scale = Math.sqrt(((gamepad1.right_stick_y) * (gamepad1.right_stick_y))
                    + ((gamepad1.right_stick_x) * (gamepad1.right_stick_x))) ;
            double turnScale = gamepad1.left_stick_x;
            telemetry.addData("Scale", scale);
            telemetry.addData("turnScale", turnScale);
            telemetry.update();
            rd.moveTeleop(angle, scale, turnScale);
            if (gamepad1.a) {

                rh.level.setPosition(0);
            } else if (gamepad1.b) {

                rh.level.setPosition(.5);
            } 
            else if (gamepad1.x) {

                rh.level.setPosition(1);
            } else if (gamepad1.y) {

                
            }

           // rd.startIntake(gamepad1.left_trigger);

           // rh.intakeMotorLeft.setPower(gamepad1.left_trigger);
            //rh.intakeMotorRight.setPower(-gamepad1.right_trigger);

        }

    }

}
