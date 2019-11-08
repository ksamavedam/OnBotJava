package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "testslides2 (Blocks to Java)", group = "")
public class testslides2 extends LinearOpMode {

  private DcMotor slides;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    slides = hardwareMap.dcMotor.get("slides");

    // Put initialization blocks here.
    while (gamepad1.a) {
      slides.setPower(0.5);
    }
    while (gamepad1.b) {
      slides.setPower(-0.5);
    }
    waitForStart();
  }
}
