package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ArmTest2 (Blocks to Java)", group = "")
public class ArmTest2 extends LinearOpMode {

  private DcMotor armMotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    armMotor = hardwareMap.dcMotor.get("armMotor");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      armMotor.setTargetPosition(1426);
      while (opModeIsActive()) {
        if (gamepad1.a) {
          armMotor.setPower(1);
        }
        if (gamepad1.b) {
          armMotor.setTargetPosition(20);
          armMotor.setPower(0.5);
        }
        telemetry.addData("Arm Position", armMotor.getCurrentPosition());
        // Put loop blocks here.
        telemetry.update();
      }
    }
  }
}
