package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GrizzlyPear implements FourWheeledRobot {
    private WrappedDCMotor motors[];
    //private WrappedDCMotor dunkMotor;

    private final int frontLeftDrive = 2;
    private final int rearLeftDrive = 1;
    private final int rearRightDrive = 0;
    private final int frontRightDrive = 3;

    private final double PROPORTIONAL_CONSTANT = 0.2;
    private final double WHEEL_RADIUS = 4;
    private final double TICKS_PER_ROTATION = 537.6;

    private final double MAX_MOTOR_RATE = 0.6;  // TODO: MEASURE REAL VALUE!!!

    public GrizzlyPear(HardwareMap hardwareMap) {
        motors = new WrappedDCMotor[4];

        for (int i = 0; i != 4; ++i) {
            motors[i] = new WrappedDCMotor(
                    hardwareMap.get(DcMotor.class, "motor" + Integer.toString(i + 1)),
                    PROPORTIONAL_CONSTANT,
                    TICKS_PER_ROTATION);
        }

        //dunkMotor = new WrappedDCMotor(hardwareMap.get(DcMotor.class, "dunker"),
               // 0.08,
               // TICKS_PER_ROTATION);
       // dunkMotor.setDirection(DcMotor.Direction.REVERSE);

        motors[rearRightDrive].setDirection(DcMotor.Direction.REVERSE);
        motors[rearLeftDrive].setDirection(DcMotor.Direction.FORWARD);
        motors[frontRightDrive].setDirection(DcMotor.Direction.REVERSE);
        motors[frontLeftDrive].setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public int getFrontLeftIndex() {
        return frontLeftDrive;
    }

    @Override
    public int getFrontRightIndex() {
        return frontRightDrive;
    }

    @Override
    public int getRearLeftIndex() {
        return rearLeftDrive;
    }

    @Override
    public int getRearRightIndex() {
        return rearRightDrive;
    }

    @Override
    public WrappedDCMotor getDrive(int index) {
        return motors[index];
    }

    @Override
    public void update() {
        for (WrappedDCMotor motor : motors) {
            motor.proportionalUpdate();
        }
    }

    @Override
    public double getTicksPerInch() {
        return TICKS_PER_ROTATION / (2 * Math.PI * WHEEL_RADIUS);
    }

    public double getMaxMotorRate() {
        return MAX_MOTOR_RATE;
    }

  //  public WrappedDCMotor getDunkMotor() {
     //   return dunkMotor;
   // }
    
    public WrappedDCMotor getFrontLeftDrive() {
        return getDrive(getFrontLeftIndex());
    }

    public WrappedDCMotor getFrontRightDrive() {
        return getDrive(getFrontRightIndex());
    }

    public WrappedDCMotor getRearLeftDrive() {
        return getDrive(getRearLeftIndex());
    }

    public WrappedDCMotor getRearRightDrive() {
        return getDrive(getRearRightIndex());
    }

}
