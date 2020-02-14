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
    public void moveArm(double targetPos){

        hw.armRight.setPosition(1-targetPos);
        hw.armLeft.setPosition(targetPos);
        if(targetPos == hw.startPos())
            hw.level.setPosition(0.05);
        else    
            hw.level.setPosition(targetPos+hw.levelConstant());

    }

   
public void proportionalTurn(double targetAngle){

    Orientation angles= hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double delta = 0;
    mRunTime= new ElapsedTime();
    mRunTime.reset();
    angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    delta = targetAngle - angles.firstAngle;

    while(delta>0.01 || delta < -0.01) {
    angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    delta = targetAngle - angles.firstAngle;
    //if(delta!=0){

        if(delta>180){

            delta = delta - 360;
        }
        else if(delta <-180){

            delta += 360;
        }
        double power = (delta * .025);
        power = Math.max((Math.min(Math.abs(power), 75)),.15) * (Math.abs(power) / power) ;
        setPower(-power, -power, power, power);
    }
    setPower(0,0,0,0);
    resetEncoders();
}

    public void proportionalTurn(double targetAngle, double time){

        Orientation angles= hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double delta = 0;
    mRunTime= new ElapsedTime();
    mRunTime.reset();
    angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    delta = targetAngle - angles.firstAngle;

    while(mRunTime.time() < time) {
    angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    delta = targetAngle - angles.firstAngle;
    //if(delta!=0){

        if(delta>180){

            delta = delta - 360;
        }
        else if(delta <-180){

            delta += 360;
        }
        double power = (delta * .025);
        power = Math.max((Math.min(Math.abs(power), 75)),.15) * (Math.abs(power) / power) ;
        setPower(-power, -power, power, power);
    }
    setPower(0,0,0,0);
    resetEncoders();
    }

    public void moveCorrect(double power, double angleOfMove, double targetHeading, double time ){

        Orientation angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double[] powers = new double[4];
        ElapsedTime mRuntime = new ElapsedTime();
        while(mRuntime.time() < time){

            powers = move(angleOfMove, power, propPower(targetHeading, angles));
            hw.tlMotor.setPower(powers[0]);
            hw.blMotor.setPower(powers[1]);
            hw.brMotor.setPower(powers[2]);
            hw.trMotor.setPower(powers[3]);
        }
    }

    private double propPower(double targetAngle, Orientation angles){

        angles = hw.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double delta = targetAngle - angles.firstAngle;
        //if(delta!=0){

        if(delta>180){

            delta = delta - 360;
        }
        else if(delta <-180){

            delta += 360;
        }
        double power = (delta * .025);
        power = Math.max((Math.min(Math.abs(power), 75)),.15) * (Math.abs(power) / power) ;
        return power;
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

    public void setPower(double tlPower, double blPower, double brPower, double trPower) {

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

    public void scoreFoundationRed( double angle1, double angle2){

        proportionalTurn(angle1, .75);
        resetEncoders();
        moveDist(RobotDrive.Direction.FORWARD, 35, 1);
        proportionalTurn(angle2, .75);
        resetEncoders();
        lockFoundation("unlock");
    }

    public void lockFoundation(String toLock){

        if(toLock.equalsIgnoreCase("Lock")){

            hw.f_servoRight.setPosition(1);
            hw.f_servoLeft.setPosition(.5);
        }
        else{

            hw.f_servoRight.setPosition(.5);
            hw.f_servoLeft.setPosition(1);
        }
    }
}
