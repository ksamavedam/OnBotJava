package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class WrappedDCMotor {

    private final DcMotor motor;
    private final double proportionalConstant;

    private final ElapsedTime timer;

    private double rate = 0; // ticks / milli

    private double[] previousTimes;
    private double[] previousTicks;

    private int STEPS = 2;

    public final double ticksPerRotation;


    public WrappedDCMotor(DcMotor motor, double proportionalConstant, double ticksPerRotation) {
        this.motor = motor;
        this.proportionalConstant = proportionalConstant;
        this.ticksPerRotation = ticksPerRotation;
        timer = new ElapsedTime();

        motor.setPower(0);

        previousTicks = new double[STEPS];
        previousTimes = new double[STEPS];
        for (int i = 0; i != STEPS; ++i) {
            previousTimes[i] = timer.milliseconds();
            previousTicks[i] = motor.getCurrentPosition();
        }
    }

    public int getEncoderCount() {
        return motor.getCurrentPosition();
    }

    public void setRawPower(double power) {
        motor.setPower(power);
    }

    public void setRate(double ticksPerMilli) {
//        motor.setPower(motor.getPower() * ticksPerMilli / rate);
        rate = ticksPerMilli;
        proportionalUpdate();
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public void proportionalUpdate() {
        double timeDelta = timer.milliseconds() - previousTimes[0];
        double tickDelta = getEncoderCount() - previousTicks[0];
        double currRate = tickDelta / timeDelta;
        double rateDelta = rate - currRate;
        double newPower = motor.getPower() + rateDelta * proportionalConstant;
        double cappedNewPower = Math.min(1, Math.max(-1, newPower));
        motor.setPower(cappedNewPower);

        motor.setPower(rate);

        for (int i = 0; i != STEPS - 1; ++i) {
            previousTimes[i] = previousTimes[i + 1];
            previousTicks[i] = previousTicks[i + 1];
        }
        previousTimes[STEPS - 1] = timer.milliseconds();
        previousTicks[STEPS - 1] = getEncoderCount();
    }
}
