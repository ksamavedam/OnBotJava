
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the FourWheeledRobot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "TeleOpControl", group = "TeleOpModes")
public class TeleOpControl extends LinearOpMode {

    private GrizzlyPear robot;
    private MechanoMotion motion;
    private Minerals minerals;
    DcMotor extendMotor = null;
    DcMotor latchMotor = null;
    DcMotor intakeArm = null;
    DcMotor dumpMotor = null;
    CRServo intake = null;
    
    @Override
    public void runOpMode() {
        /*robot = new GrizzlyPear(hardwareMap);
        motion = new MechanoMotion(robot);
        minerals = new Minerals(telemetry, gamepad1, gamepad2, robot.getDunkMotor());
        extendMotor = hardwareMap.get(DcMotor.class, "extendMotor");
        latchMotor = hardwareMap.get(DcMotor.class, "latchMotor");
        intakeArm = hardwareMap.get(DcMotor.class, "intakeArm");
        dumpMotor = hardwareMap.get(DcMotor.class, "dumpMotor");
        intake = hardwareMap.get(CRServo.class, "intake");
        waitForStart();


        while (opModeIsActive()) {
            boolean flag = true;
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            // Krishna Added for Harbin Testing
                dumpMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                dumpMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                dumpMotor.setPower(gamepad1.right_stick_x * 0.75);
                if(1)
                    continue;
            // Krishna - Done

            //Moving
            Motion movement = motion.rightwardsMotion(gamepad1.right_stick_x)
                    .add(motion.forwardMotion(-gamepad1.right_stick_y))
                    .add(motion.rotationMotion(gamepad1.left_stick_x));
            if (gamepad1.dpad_left) {
                movement = movement.add(motion.forwardMotion(-0.3));
            }
            if (gamepad1.dpad_right) {
                movement = movement.add(motion.forwardMotion(0.3));
            }
            
            if (gamepad1.dpad_up) {
                movement = movement.add(motion.rightwardsMotion(0.7));
            }
            if (gamepad1.dpad_down) {
                movement = movement.add(motion.rightwardsMotion(-0.7));
            }
            motion.executeRate(movement);
            
            // Intake Arm(out)
            if (gamepad2.a) {
                if (flag==true) {
                    flag=false;
                    flipOut();
                }
                flag=true;
            }
            // Intake Arm(in)
            if (gamepad2.b) {
                if (flag==true) {
                    flag=false;
                    deposit();
                }
                flag=true;
            }



            
            //Scoring Basket
            if(gamepad1.a){
                dumpMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                dumpMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                dumpMotor.setTargetPosition(1758);
                dumpMotor.setPower(.5);
                
            }
            if(gamepad1.b){
                dumpMotor.setPower(1);
            }    
            
            if(gamepad1.x){
                dumpMotor.setPower(-.5);
            }
            
            // Intake
            if (gamepad1.right_trigger > 0) {
                intake.setPower(.9);
            }
            // Outtake (these may be switched)
            else if (gamepad1.left_trigger > 0) {
                intake.setPower(-.9);
            }
            else {
                intake.setPower(0);
            }
            
            //Extensions
            extendMotor.setPower(gamepad2.right_stick_x);
            
            //Latching
            latchMotor.setPower(gamepad2.left_stick_y);
        }

}
     public void deposit() {
          double offset = intakeArm.getCurrentPosition();
          while (opModeIsActive() && 
          intakeArm.getCurrentPosition()-offset < 108 && !gamepad1.x) 
          {
            intakeArm.setPower(-1);
            telemetry.addData("Encoders", intakeArm.getCurrentPosition()-offset);
            telemetry.update();
            
          }
          intakeArm.setPower(0);
        }
        public void flipOut() {
           double offset = intakeArm.getCurrentPosition();
           while (opModeIsActive() && intakeArm.getCurrentPosition()-offset > -95 && !gamepad1.x) {
            intakeArm.setPower(.7);
            telemetry.addData("Encoders", intakeArm.getCurrentPosition()-offset);
            telemetry.update();
            
           }
        */
        }
        
}
