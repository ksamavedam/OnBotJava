package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "testslides (Blocks to Java)", group = "")
public class testslides extends LinearOpMode {

  private DcMotor slides;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    slides = hardwareMap.dcMotor.get("slides");

    // Put initialization blocks here.
    waitForStart();
    if (false) {
      slides.setPower(0.5);
    }
  }
}
