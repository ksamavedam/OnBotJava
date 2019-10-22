package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
@TeleOp(name = "JustDrivingLol", group = "TeleOpModes")

public class JustDrivingLol extends LinearOpMode{
    private GrizzlyPear robot;
    private MechanoMotion motion;
    DcMotor motor1= null;
    DcMotor motor2= null;
    DcMotor motor3= null;
    DcMotor motor4= null;
    
    @Override
    public void runOpMode() {
        robot = new GrizzlyPear(hardwareMap);
        motion = new MechanoMotion(robot);
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor4 = hardwareMap.get(DcMotor.class, "motor4");
        waitForStart();
        
        while (opModeIsActive()) {
           telemetry.addData("Status", "Initialized");
           telemetry.update();
           
           Motion movement = motion.rightwardsMotion(gamepad1.right_stick_x)
                    .add(motion.forwardMotion(-gamepad1.right_stick_y))
                    .add(motion.rotationMotion(gamepad1.left_stick_x));
            if (gamepad1.dpad_left) {
                movement = movement.add(motion.forwardMotion(-0.5));
            }
            if (gamepad1.dpad_right) {
                movement = movement.add(motion.forwardMotion(0.5));
            }
            
            if (gamepad1.dpad_up) {
                movement = movement.add(motion.rightwardsMotion(0.7));
            }
            if (gamepad1.dpad_down) {
                movement = movement.add(motion.rightwardsMotion(-0.7));
            }
            motion.executeRate(movement);
        }
    }
}
