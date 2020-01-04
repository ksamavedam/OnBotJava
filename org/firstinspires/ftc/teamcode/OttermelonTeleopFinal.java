
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

@TeleOp(name = "OtttermelonTeleopFinal", group = "TeleOpModes")

public class OttermelonTeleopFinal extends LinearOpMode {
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
            double s1Pos=0.025;
            /************************************************************************************ */
            //DRIVER 1 CONTROLS
            //*************************************************************************************/

            //Drivetrain (Left Joystick - Rotate, Right Joystick - Moving)
            Orientation angles= rh.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double offset = -1 * Math.toRadians(angles.firstAngle);
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

           // angle += offset;
            telemetry.addData("offset", offset);
            telemetry.addData("Angle: ", Math.toDegrees(angle));

            //Desired power of moving
            double scale = Math.sqrt(((gamepad1.right_stick_y) * (gamepad1.right_stick_y))
                    + ((gamepad1.right_stick_x) * (gamepad1.right_stick_x))) ;

            //Rotating one
            double turnScale = gamepad1.left_stick_x;
            telemetry.addData("Scale", scale);
            telemetry.addData("turnScale", turnScale);
            telemetry.update();

            //Actual call of the move function
            rd.moveTeleop(angle, scale, turnScale);

            //Foundation Servos Down (Button A) Grab
            if (gamepad1.a) {
                rh.f_servoLeft.setPosition(1);
                rh.f_servoRight.setPosition(1);
            } 
            
            //Foundation Servos Up (Button B) Release
            else if (gamepad1.b) {
                rh.f_servoLeft.setPosition(.5);
                rh.f_servoRight.setPosition(.5);
            } 

            /***************************************************************************** */
            //DRIVER 2 CONTROLS
            /***************************************************************************** */
            
            //Normal Speed Slides (Left Joystick)
            rh.slideLeft.setPower(gamepad2.left_stick_y);
            rh.slideRight.setPower(gamepad2.left_stick_y);

            //Intake (Left Trigger)
            if (gamepad2.left_trigger > 0){
                rh.intakeMotorRight.setPower(.4);
                rh.intakeMotorLeft.setPower(-.4);
            }

            //Outtake (Right Trigger)
            else if (gamepad2.right_trigger > 0){
                //Getting the gripper out of the way of the block
                rh.gripper.setPosition(0);
                rh.intakeMotorRight.setPower(-.45);
                rh.intakeMotorLeft.setPower(.45);
            }

            //Stop Intake (Passive)
            else {
                rh.intakeMotorRight.setPower(0);
                rh.intakeMotorLeft.setPower(0);
            }

            //Grip the stone (Button X)
            if (gamepad2.x){
                rh.gripper.setPosition(.7);
            }

            //Drop the stone (Button Y)
            else if (gamepad2.y){
                rh.gripper.setPosition(0);
            }

            //Arm in the robot (Button A) TEST
            if (gamepad2.a){
                s1Pos = 0;
            }

            //Arm in scoring position (Button B) TEST
            else if (gamepad2.b){
                s1Pos = .6;
            }

            rh.armRight.setPosition(1-s1Pos);
            rh.armLeft.setPosition(s1Pos);
            rh.level.setPosition(s1Pos+.02);
        }

    }

}
