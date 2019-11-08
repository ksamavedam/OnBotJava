package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Minerals {

    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;
    private final WrappedDCMotor dunkMotor;
    public Minerals(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, WrappedDCMotor dunkMotor) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.dunkMotor = dunkMotor;
    }

    public void dunk() {
        while (dunkMotor.getEncoderCount() < 75 && !gamepad2.x) {
            telemetry.addData("Status", "Dunking");
            telemetry.addData("Angle", dunkMotor.getEncoderCount());
            telemetry.update();
            dunkMotor.setRawPower(0.7);
        }
        dunkMotor.setRawPower(-0.03);
        while (dunkMotor.getEncoderCount() > 50) { }
        while (dunkMotor.getEncoderCount() > 30 && !gamepad2.x) {
            telemetry.addData("Status", "Dunking");
            telemetry.addData("Angle", dunkMotor.getEncoderCount());
            telemetry.update();
            dunkMotor.setRawPower(0.15);
        }
        dunkMotor.setRawPower(0);
    }
}
