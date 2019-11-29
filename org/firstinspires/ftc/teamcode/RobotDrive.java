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

        double[] powers = new double[4];
        powers = move(angle, power, 0);
        hw.tlMotor.setPower(powers[0]);
        hw.blMotor.setPower(powers[1]);
        hw.brMotor.setPower(powers[2]);
        hw.trMotor.setPower(powers[3]);
    }

    public void moveTeleop(double angle, double scale, double turnScale) {

        double[] powers = new double[4];
        powers = move(angle, scale, turnScale);

        hw.tlMotor.setPower(powers[0]);
        hw.blMotor.setPower(powers[1]);
        hw.brMotor.setPower(powers[2]);
        hw.trMotor.setPower(powers[3]);
    }

   /* public void proportionalTurn(double targetAngle, double time) {

        mRunTime= new ElapsedTime();
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
    }*/

    public void proportionalTurn(double targetAngle, double time) {

        Orientation angles= hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        mRunTime= new ElapsedTime();
        double direction = 1;
        if (targetAngle > 180) {
            targetAngle=360-targetAngle;
            direction = -1;
        }

        double delta = 0;

        mRunTime.reset();
        while (mRunTime.time() < time) {

            angles= hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angle= angles.firstAngle;
            if(angles.firstAngle<0){

                angle=Math.abs(angles.firstAngle);
            }
            delta = (targetAngle - angle);
            double power = (delta * .025);
            power = (Math.min(Math.abs(power), .75)) * (Math.abs(power) / power) ;
            setPower(-power* direction, -power* direction, power* direction, power* direction);
        }

        setPower(0, 0, 0, 0);
    }

    public void turn(double targetAngle){
        Orientation angles= hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double direction = .75;
        if (targetAngle > 180) {
            direction = -1*direction;
            targetAngle = targetAngle - 360;
        }
        while(targetAngle-angles.firstAngle>0){
            angles= hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            setPower(-direction, -direction, direction, direction);
        }
        setPower(0, 0, 0, 0);
    }

    private double getAngularOriFirst() {
        Orientation angles;
        angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle);
    }

    public void startIntake(double power){

        hw.intakeMotorLeft.setPower(power);
        hw.intakeMotorRight.setPower(-power);
    }

    private void setPower(double tlPower, double blPower, double brPower, double trPower) {

        hw.tlMotor.setPower(tlPower);
        hw.blMotor.setPower(blPower);
        hw.brMotor.setPower(brPower);
        hw.trMotor.setPower(trPower);
    }

    private double getMaxPower(double a, double b, double c, double d) {

        a = Math.abs(a);
        b = Math.abs(b);
        c = Math.abs(c);
        d = Math.abs(d);
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
    private double[] move(double angle, double scale, double turnScale) {
        double topLeft = 0;
        double bottomLeft = 0;
        double bottomRight = 0;
        double topRight = 0;
        double maxPower = 0;
        double[] powers = new double[4];

        // DP
        // There is a problem with the scheme below. Let's say you want to make a turn
        // very fast, but while turning
        // also move SLOW along a direction. The slow translation implies a low scale
        // number. In the scheme below
        // because scale multiplies to turn power also, the turn will become slow as
        // well. I am suggesting a modification
        // in Slack.
        if (scale != 0.0) {
            topLeft = (Math.cos(angle)) + -1 * (Math.sin(angle)) + turnScale * 1;
            bottomLeft = (Math.cos(angle)) + (Math.sin(angle)) + turnScale * 1;
            bottomRight = (Math.cos(angle)) + -1 * (Math.sin(angle)) + turnScale * -1;
            topRight = (Math.cos(angle)) + (Math.sin(angle)) + turnScale * -1;

            maxPower = getMaxPower(topLeft, bottomLeft, bottomRight, topRight);

            topLeft = topLeft / maxPower;
            bottomLeft = bottomLeft / maxPower;
            bottomRight = bottomRight / maxPower;
            topRight = topRight / maxPower;

            topLeft *= scale;
            bottomLeft *= scale;
            bottomRight *= scale;
            topRight *= scale;

            powers[0] = topLeft;
            powers[1] = bottomLeft;
            powers[2] = bottomRight;
            powers[3] = topRight;
        } else {
            powers[0] = 1 * turnScale;
            powers[1] = 1 * turnScale;
            powers[2] = -1 * turnScale;
            powers[3] = -1 * turnScale;

        }
        return powers;
    }
}
