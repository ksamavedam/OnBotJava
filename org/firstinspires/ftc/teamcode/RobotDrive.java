package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotHardware;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotDrive {
    // Constants for moving in each direction
    double ticksToInchV = 0;
    double ticksToInchH = 0;
    double ticksToInchR = 0;
    double ticksToInchD = 0;
    Orientation angles;
    RobotHardware hw;
    ElapsedTime mRunTime;

    class Powers {
        double topLeft;
        double topRight; 
        double bottomLeft;
        double bottomRight; 
        Powers() {
            topLeft     = 0; 
            topRight    = 0; 
            bottomLeft  = 0;
            bottomRight = 0;
        }
    };

    RobotDrive(RobotHardware rhw) {
        hw = rhw;

        ticksToInchV = hw.getTicksToInchV();
        ticksToInchH = hw.getTicksToInchH();
        ticksToInchR = hw.getTicksToInchR();
        ticksToInchD = hw.getTicksToInchD();

        resetEncoders();
    }

    enum Direction {
        FORWARD, REVERSE, RIGHT, LEFT
    }

    enum Rotation {
        CLOCKWISE, ANTICLOCKWISE
    }

    public void moveDist(RobotDrive.Direction dir, double distance, double power) {

        switch (dir) {
        case FORWARD:
            moveImpl(distance, 0, power);
            break;
        case REVERSE:
            moveImpl(distance, 180, power);
            break;
        case RIGHT:
            moveImpl(distance, 270, power);
            break;
        case LEFT:
            moveImpl(distance, 90, power);
            break;

        }
    }

    /*
     * public void rotate(RobotDrive.Rotation dir, double speed)){ switch (dir) {
     * case CLOCKWISE: moveEnc(0, 0, speed, distance); break; case ANTICLOCKWISE:
     * moveEnc(180, 0, -1*speed, distance); break; }
     */

    private void moveImpl(double dist, double angle, double power) {

        double targetPosition = 0;

        if (angle == 0) {

            targetPosition = ticksToInchV * dist;

        } else if (angle == 180) {
            targetPosition = ticksToInchV * dist;

        } else if (angle == 90) {
            targetPosition = ticksToInchH * dist;

        } else if (angle == 270) {
            targetPosition = ticksToInchH * dist;

        } else {

            throw new IllegalArgumentException("Ya can't be enterin an angle other than 0, 90 ,180 or 270.");
        }
        angle = Math.toRadians(angle);
        moveAngle(angle, power);
        while (Math.abs(hw.tlMotor.getCurrentPosition()) < targetPosition) {

        }
        moveAngle(0, 0);
        resetEncoders();
    }

    private void moveAngle(double angle, double power) {

        Powers p;
        p = move(angle, power, 0);
        hw.tlMotor.setPower(p.topLeft);
        hw.blMotor.setPower(p.bottomLeft);
        hw.brMotor.setPower(p.bottomRight);
        hw.trMotor.setPower(p.topRight);
    }

    public void moveTeleop(double angle, double scale, double turnScale) {

        Powers p;
        p = move(angle, scale, turnScale);

        hw.tlMotor.setPower(p.topLeft);
        hw.blMotor.setPower(p.bottomLeft);
        hw.brMotor.setPower(p.bottomRight);
        hw.trMotor.setPower(p.topRight);
    }

    public void proportionalTurn(double targetAngle, double time) {

        double direction = 1;
        if (targetAngle > 180) {
            direction = -1;
            targetAngle = targetAngle - 360;
        }

        double delta = (targetAngle - getAngularOriFirst());

        mRunTime.reset();
        while (mRunTime.time() < time) {
            delta = (targetAngle - getAngularOriFirst());
            double power = (delta * .05);
            power = (Math.min(Math.abs(power), .75)) * (Math.abs(power) / power) * direction;
            setPower(power, power, -power, -power);
        }

        setPower(0, 0, 0, 0);
    }

    private double getAngularOriFirst() {
        Orientation angles;
        angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle);
    }

    public void setPower(double tlPower, double blPower, double brPower, double trPower) {

        hw.tlMotor.setPower(tlPower);
        hw.blMotor.setPower(blPower);
        hw.brMotor.setPower(brPower);
        hw.trMotor.setPower(trPower);
    }

    private double getMaxPower(Powers p) {

        double a = Math.abs(p.topLeft);
        double b = Math.abs(p.topRight);
        double c = Math.abs(p.bottomLeft);
        double d = Math.abs(p.bottomRight);
        return (Math.max(a, Math.max(b, Math.max(c, d))));

    }

    // resets after each move
    public void resetEncoders() {
        hw.tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hw.trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hw.tlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        hw.blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        hw.brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        hw.trMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }

    // returns powers of each motor to move at a certain angle
    private Powers move(double angle, double scale, double turnScale) {
        Powers p = new Powers() ;
        double maxPower = 0;
        
        // DP
        // There is a problem with the scheme below. Let's say you want to make a turn
        // very fast, but while turning
        // also move SLOW along a direction. The slow translation implies a low scale
        // number. In the scheme below
        // because scale multiplies to turn power also, the turn will become slow as
        // well. I am suggesting a modification
        // in Slack.
        if (scale != 0.0) {
             
            p.topLeft = (Math.cos(angle)) + -1 * (Math.sin(angle)) + turnScale * 1;
            p.bottomLeft = (Math.cos(angle)) + (Math.sin(angle)) + turnScale * 1;
            p.bottomRight = (Math.cos(angle)) + -1 * (Math.sin(angle)) + turnScale * -1;
            p.topRight = (Math.cos(angle)) + (Math.sin(angle)) + turnScale * -1;

            maxPower = getMaxPower(p);

            p.topLeft     /=  maxPower;
            p.bottomLeft  /=  maxPower;
            p.bottomRight /= maxPower;
            p.topRight    /=  maxPower;

            p.topLeft     *= scale;
            p.bottomLeft  *= scale;
            p.bottomRight *= scale;
            p.topRight    *= scale;
        } else {
            p.topLeft     = turnScale;
            p.bottomLeft  = turnScale ;
            p.bottomRight = turnScale * -1;
            p.topRight    = turnScale  * -1;
        }
        return p;
    }
}
