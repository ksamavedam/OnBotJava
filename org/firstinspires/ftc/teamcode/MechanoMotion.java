package org.firstinspires.ftc.teamcode;

public class MechanoMotion {

    private FourWheeledRobot robot;

    public MechanoMotion(FourWheeledRobot robot) {
        this.robot = robot;
    }

    // Direction measured as clockwise offset from forward direction relative to robot (in deg)
    // rate is from 0 -> 1
    public void moveRate(double rate, double degrees) {
        double radians = Math.toRadians(degrees);
        Motion motion = forwardMotion(rate * Math.cos(radians))
                .add(rightwardsMotion(rate * Math.sin(radians)));

        executeRate(motion);
    }


    public void moveDistance(double distance, double rate, double degrees) {
        double radians = Math.toRadians(degrees);
        Motion motion = forwardMotion(distance * robot.getTicksPerInch() * Math.cos(radians))
                .add(rightwardsMotion(distance * robot.getTicksPerInch() * Math.sin(radians)));

        executeDistance(motion, rate);
    }

    public void rotate(double degrees) {

    }

    public void executeRate(Motion motion) {
        for (int i = 0; i != 4; ++i) {
//            robot.getDrive(i).setRate(motion.getValue(i));
            robot.getDrive(i).setRate(motion.getValue(i) * robot.getMaxMotorRate());
        }
    }

    public void executeDistance(Motion motion, double rate) {
        // each element of the motion is a number of ticks!

        WrappedDCMotor motor = null;
        for (int i = 0; i != 4; ++i) {
            if (motion.getValue(i) != 0) {
                motor = robot.getDrive(i);
            }
        }
        if (motor == null) {
            return;
        }
    }

    public Motion forwardMotion(double x) {
        double[] values = new double[4];
        values[robot.getFrontLeftIndex()] = x;
        values[robot.getFrontRightIndex()] = x;
        values[robot.getRearLeftIndex()] = x;
        values[robot.getRearRightIndex()] = x;
        return new Motion(values);
    }

    public Motion rightwardsMotion(double x) {
        double[] values = new double[4];
        values[robot.getFrontLeftIndex()] = x;
        values[robot.getFrontRightIndex()] = -x;
        values[robot.getRearLeftIndex()] = -x;
        values[robot.getRearRightIndex()] = x;
        return new Motion(values);
    }

    public Motion rotationMotion(double x) {
        double[] values = new double[4];
        values[robot.getFrontLeftIndex()] = -x;
        values[robot.getFrontRightIndex()] = x;
        values[robot.getRearLeftIndex()] = -x;
        values[robot.getRearRightIndex()] = x;
        return new Motion(values);
    }
}
