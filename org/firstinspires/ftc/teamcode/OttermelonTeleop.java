
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

@TeleOp(name = "OtttermelonTeleopLift", group = "TeleOpModes")

public class OttermelonTeleop extends LinearOpMode {
    private BNO055IMU imu;

    public ElapsedTime mRunTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        RobotHardware rh = null;
        RobotDrive rd = null;
        rh = new RobotHardware("AppleBee", hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            
 
                rh.rMotor.setPower(gamepad1.right_stick_y * 0.99);
                rh.lMotor.setPower(gamepad1.right_stick_y * -0.99);
 
      
           // rd.startIntake(gamepad1.left_trigger);

           // rh.intakeMotorLeft.setPower(gamepad1.left_trigger);
            //rh.intakeMotorRight.setPower(-gamepad1.right_trigger);

        }

    }

}
