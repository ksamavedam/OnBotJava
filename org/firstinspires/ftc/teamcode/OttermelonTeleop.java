
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
    RobotHardware rh = null;
    RobotDrive rd = null;
    public ElapsedTime mRunTime = new ElapsedTime();

    @Override
    public void runOpMode() {

        rh = new RobotHardware("OtterMelon", hardwareMap);
        rd = new RobotDrive(rh);
        waitForStart();
        while (opModeIsActive()) {

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double offset = -1 * Math.toRadians(angles.firstAngle);
            double angle = calcAngleGP1_RtStick();
            angle += offset;

            telemetry.addData("offset", offset);
            telemetry.addData("Angle: ", Math.toDegrees(angle));

            double scale = calcScaleGP1_RtStick();

            double turnScale = gamepad1.left_stick_x;
            telemetry.addData("Scale", scale);
            telemetry.addData("turnScale", turnScale);
            telemetry.update();

            rd.moveTeleop(angle, scale, turnScale);

            setMotPwr_GP1_abxy();

        }

    }

    private void setMotPwr_GP1_abxy(){
        
        if (gamepad1.a) {
            rh.tlMotor.setPower(.5);
        } else if (gamepad1.b) {
            rh.blMotor.setPower(.5);
        } else if (gamepad1.x) {
            rh.brMotor.setPower(.4);
        } else if (gamepad1.y) {
            rh.trMotor.setPower(.4);
        }
    }

    private double calcScaleGP1_RtStick() {
        double scale = 0;
        double x = gamepad1.right_stick_x; 
        double y = gamepad1.right_stick_y; 
        return (Math.sqrt(((y) * (y)) + ((x) * (x))) * .75);
    }

    private double calcAngleGP1_RtStick() {
        double angle = 0;
        double x = gamepad1.right_stick_x; 
        double y = gamepad1.right_stick_y; 
        
        if (x > 0 && y <= 0) {
            angle = Math.atan(-y / x) + 3 * Math.PI / 2;
        } else if (x >= 0 && y > 0) {
            angle = Math.atan(x / y) + Math.PI;
        } else if (x < 0 && y >= 0) {
            angle = Math.atan(y / -x) + Math.PI / 2;
        } else if (x <= 0 && y < 0) {
            angle = Math.atan(x / y);
        } 
        return angle;
    }
}
