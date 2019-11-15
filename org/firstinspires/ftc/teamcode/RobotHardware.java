package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public BNO055IMU imu;
    public DcMotor tlMotor;
    public DcMotor blMotor;
    public DcMotor brMotor;
    public DcMotor trMotor;
    public DcMotor grabberMotor;
    BNO055IMU.Parameters imuParameters;

    Servo foundationServo;
    Servo grabber;
    Blinker expansion_Hub_2;
    double ticksToInchV = 32.0;
    double ticksToInchH = 37.0;
    double ticksToInchR = 15.0;
    double ticksToInchD = 49.0;
    HardwareMap hwMap = null;

    public RobotHardware(String name, HardwareMap hw) {
        if (name == "OtterMelon") {
            ticksToInchV = 32.0;
            ticksToInchH = 37.0;
            ticksToInchR = 15.0;
            ticksToInchD = 49.0;
        } else if (name == "{Ri3D}") {
            // TBD
            ticksToInchV = 32.0;
            ticksToInchH = 37.0;
            ticksToInchR = 15.0;
            ticksToInchD = 49.0;
        }
        hwMap = hw;
        tlMotor = hwMap.get(DcMotor.class, "topLeft");
        blMotor = hwMap.get(DcMotor.class, "bottomLeft");
        brMotor = hwMap.get(DcMotor.class, "bottomRight");
        trMotor = hwMap.get(DcMotor.class, "topRight");
        grabberMotor = hwMap.get(DcMotor.class, "grabber arm");
        foundationServo = hwMap.get(Servo.class, "foundationServo");
        grabber = hwMap.get(Servo.class, "grabber");

        imu = hwMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);

    }
    public double getTicksToInchV() {return ticksToInchV; }
    public double getTicksToInchH() {return ticksToInchH; }
    public double getTicksToInchR() {return ticksToInchR; }
    public double getTicksToInchD() {return ticksToInchD; }

    
}
