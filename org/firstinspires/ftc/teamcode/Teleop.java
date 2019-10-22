/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp", group="Linear Opmode")

public class Teleop extends LinearOpMode {
    CRServo intakeArm = null;
    Servo arm = null;
    @Override
    public void runOpMode() {
        RobotHardware rh = new RobotHardware("GrizzlyPear", hardwareMap);
        MecanumMath mwm = new MecanumMath(rh);
        intakeArm = hardwareMap.get(CRServo.class, "intake");
        arm = hardwareMap.get(Servo.class, "arm");
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double Power1;
            double Power2;
            double Power3;
            double Power4;
            double speed;
            double direction;
            double rotationSpeed;
            double radius;
            double radiusAngle;
            double wheelRadius;
            double dampSpeed = 0.4;
            double left = -gamepad2.left_stick_x;
            double leftPrevious = 0;
            double lowSpeedNeg = -.25;
            double lowSpeedPos = .25;
            double goIn = gamepad2.right_trigger;
            double goOut = -gamepad2.left_trigger;
            rotationSpeed = -gamepad1.left_stick_x;
            speed = Math.sqrt(Math.pow(gamepad1.right_stick_x, 2)+Math.pow(gamepad1.right_stick_y, 2));
            if (gamepad1.right_stick_x > 0 && gamepad1.right_stick_y <= 0) {
                direction = Math.atan(-gamepad1.right_stick_y/gamepad1.right_stick_x)+3*Math.PI/2;
            }
            else if (gamepad1.right_stick_x >= 0 && gamepad1.right_stick_y > 0) {
                direction = Math.atan(gamepad1.right_stick_x/gamepad1.right_stick_y)+Math.PI;
            }
            else if (gamepad1.right_stick_x < 0 && gamepad1.right_stick_y >= 0) {
                direction = Math.atan(gamepad1.right_stick_y/-gamepad1.right_stick_x)+Math.PI/2;
            }
            else if (gamepad1.right_stick_x <= 0 && gamepad1.right_stick_y < 0) {
                direction = Math.atan(gamepad1.right_stick_x/gamepad1.right_stick_y);
            }
            else {
                direction = 0;
            }
            
                        if (gamepad1.a) {
                intakeArm.setPower(0.70);
        telemetry.addData("Status", "Gamepad A");
        telemetry.update();
                
            }
            if (gamepad1.b) {
                intakeArm.setPower(-0.70);
            }
            if (gamepad1.x){
                intakeArm.setPower(0);
            }
            
           if(gamepad1.dpad_up){
               arm.setPosition(0.50);
           }
           if(gamepad1.dpad_down){
               arm.setPosition(-0.90);
           }
          
    

            Power1 = mwm.power1(speed, rotationSpeed, direction,1);
            Power2 = mwm.power2(speed, rotationSpeed, direction,1);
            Power3 = mwm.power3(speed, rotationSpeed, direction,1);
            Power4 = mwm.power4(speed, rotationSpeed, direction,1);
            
            rh.motor1.setPower(Power1*dampSpeed);
            rh.motor2.setPower(Power2*dampSpeed);
            rh.motor3.setPower(Power3*dampSpeed);
            rh.motor4.setPower(Power4*dampSpeed);
            
            telemetry.addData("Status", "Running");
            telemetry.update();
             
        }
    }
    
}
*/
