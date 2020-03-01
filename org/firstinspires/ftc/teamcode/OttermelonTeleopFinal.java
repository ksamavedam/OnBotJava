
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
        boolean capPos=false;

        double allignDist = 0;
        String side = "blue";
        waitForStart();
        while (opModeIsActive()) {
            double s1Pos=rh.startPos();
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

            //Desired power of moving
            double scale = Math.sqrt(((gamepad1.right_stick_y) * (gamepad1.right_stick_y))
                    + ((gamepad1.right_stick_x) * (gamepad1.right_stick_x))) ;

            //Rotating one
            double turnScale = -gamepad1.left_stick_x;

            
            if(gamepad1.left_bumper){

                rd.lockFoundation("unlock");
               // rh.f_servoLeft.setPosition(.9);
                //rh.f_servoRight.setPosition(.6);
            }
            else if(gamepad1.right_bumper){

                rd.lockFoundation("lock");
            }

            
            //All motion
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
            else if(gamepad1.left_trigger!=0){
                rd.setPower(-0.3,-0.3,0.3,0.3);
            }
            else if(gamepad1.right_trigger!=0){    
                rd.setPower(0.3,0.3,-0.3,-0.3);
            }
            else if(gamepad1.x){

                 allignDist = rh.distRight.getDistance(DistanceUnit.INCH);
                 side = "blue";
            }
            else if(gamepad1.b){

                allignDist = rh.distLeft.getDistance(DistanceUnit.INCH);
                side = "red";
            }
            else if(gamepad1.y){

                if(side.equals("red")){

                    rd.goDist(rh.distLeft, 270, .3, allignDist, false);
                }
                else {

                    rd.goDist(rh.distRight, 90, .3, allignDist, false);
                }
                
            }
            else{

                rd.moveTeleop(angle, scale, turnScale);
            }

            
        
            telemetry.addData("Left: in", "%.2f in", rh.distLeft.getDistance(DistanceUnit.INCH));
            telemetry.addData("Back: in", "%.2f in", rh.distBack.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right: in", "%.2f in", rh.distRight.getDistance(DistanceUnit.INCH));
            telemetry.addData("allign dist", allignDist);
            /***************************************************************************** */
            //DRIVER 2 CONTROLS
            /***************************************************************************** */
            
            
            //Normal Speed Slides (Left Joystick)
            rh.slideLeft.setPower(gamepad2.left_stick_y);
            rh.slideRight.setPower(gamepad2.left_stick_y);
            //Outtake (Left Trigger)
            if (gamepad2.left_trigger > 0){
                rd.startIntake(-.45);
            }

            //Intake (Right Trigger)
            else if (gamepad2.right_trigger > 0){
                //Getting the gripper out of the way of the block
                rh.intakeMotorRight.setPower(-.45);
                rh.intakeMotorLeft.setPower(.45);
                //rd.startIntake(gamepad2.right_trigger);
                rh.gripper.setPosition(.3);
            }

            //Stop Intake (Passive)
            else {
                rh.intakeMotorRight.setPower(0);
                rh.intakeMotorLeft.setPower(0);
            }

            //Grip the stone (Button X)
            if (gamepad2.x){

                rh.gripper.setPosition(.95);
            }

            //Drop the stone (Button Y)
            else if (gamepad2.y){
    
                rh.gripper.setPosition(.45);
            }
            else if(gamepad2.dpad_up){

                rh.gripper.setPosition(.8);
            }
            
            if(gamepad2.dpad_left){

                rh.tape.setPower(.7);
            }
            else if(gamepad2.dpad_right){

                rh.tape.setPower(-.9);
            }
            else{

               rh.tape.setPower(0);
            }

            //Arm in the robot (Button A) TEST
            if (gamepad2.a){
                s1Pos = rh.highScore();
            }

            //Arm in scoring position (Button B) TEST
            else if (gamepad2.b){
                s1Pos = rh.lowScore();
            }

            //capstone
            else if(gamepad2.left_bumper){

                rh.cap.setPosition(.8);
                capPos=false;
            }
            else if(gamepad2.right_bumper){

                rh.cap.setPosition(0);
            }
            rd.moveArm(s1Pos);
            telemetry.update();
    

        }

    }

}