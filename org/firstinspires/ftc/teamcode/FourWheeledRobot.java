package org.firstinspires.ftc.teamcode;

public interface FourWheeledRobot {
    int getFrontLeftIndex();
    int getFrontRightIndex();
    int getRearLeftIndex();
    int getRearRightIndex();

    WrappedDCMotor getDrive(int index);

    WrappedDCMotor getFrontLeftDrive();

    WrappedDCMotor getFrontRightDrive() ;

    WrappedDCMotor getRearLeftDrive() ;

    WrappedDCMotor getRearRightDrive() ;

    void update();

    double getTicksPerInch();
    double getMaxMotorRate();
}
