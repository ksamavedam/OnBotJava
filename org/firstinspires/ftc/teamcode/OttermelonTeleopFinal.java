
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
            double s1Pos=0.04;
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

            //Desired power of rotating
            double turnScale = gamepad1.left_stick_x*.75;
            telemetry.addData("Scale", scale);
            telemetry.addData("turnScale", turnScale);
            telemetry.update();

            //Foundation Servos Down (Button A) Grab
             if (gamepad1.x) {
                rh.f_servoLeft.setPosition(.5);
                rh.f_servoRight.setPosition(.5);
            } 
            
            //Foundation Servos Up (Button B) Release
            else if(gamepad1.b){

                rh.f_servoRight.setPosition(1);
                rh.f_servoLeft.setPosition(1);
            }

            //Alignment Right
            else if(gamepad1.left_bumper){

                rh.f_servoLeft.setPosition(1);
                rh.f_servoRight.setPosition(.5);
            }

            //Alignment Left
            else if(gamepad1.right_bumper){

                rh.f_servoLeft.setPosition(.5);
                rh.f_servoRight.setPosition(1);
            }

            //Slow motion controls
            if(gamepad1.dpad_up){
                rd.setPower(0.3,0.3,0.3,0.3);
            }
            else if(gamepad1.dpad_down){
                rd.setPower(-0.3,-0.3,-0.3,-0.3);
            }
            else if(gamepad1.dpad_right){
                rd.setPower(0.3,-0.3,0.3,-0.3);
            }
            else if(gamepad1.dpad_left){
                rd.setPower(-0.3,0.3,-0.3,0.3);
            }

            //Slow motion rotation
            else if(gamepad1.left_trigger!=0){
                rd.setPower(-0.3,-0.3,0.3,0.3);
            }
            else if(gamepad1.right_trigger!=0){    
                rd.setPower(0.3,0.3,-0.3,-0.3);
            }
            else{

                rd.moveTeleop(angle, scale, turnScale);
            }

            /***************************************************************************** */
            //DRIVER 2 CONTROLS
            /***************************************************************************** */
            
            //Normal Speed Slides (Left Joystick)
            rh.slideLeft.setPower(gamepad2.left_stick_y);
            rh.slideRight.setPower(gamepad2.left_stick_y);

            //Outtake (Left Trigger)
            if (gamepad2.left_trigger > 0){
                rd.startIntake(-.4);
            }

            //Intake (Right Trigger)
            else if (gamepad2.right_trigger > 0){
                //Getting the gripper out of the way of the block
                rh.gripper.setPosition(.3);
                rh.intakeMotorRight.setPower(-.4);
                rh.intakeMotorLeft.setPower(.4);
            }

            //Stop Intake (Passive)
            else {
                rh.intakeMotorRight.setPower(0);
                rh.intakeMotorLeft.setPower(0);
            }

            //Grip the stone (Button X)
            if (gamepad2.x){
                rh.gripper.setPosition(.8);
            }

            //Drop the stone (Button Y)
            else if (gamepad2.y){
                rh.gripper.setPosition(.3);
            }

            //Arm in the robot (Button A)
            if (gamepad2.a){
                s1Pos = .54;
            }

            //Arm in scoring position (Button B)
            else if (gamepad2.b){
                s1Pos = .65;
            }
            else if(gamepad2.dpad_down){

                s1Pos=.015;
            }

            rh.armRight.setPosition(1-s1Pos);
            rh.armLeft.setPosition(s1Pos);
            rh.level.setPosition(s1Pos+.08);
        }

    }

}