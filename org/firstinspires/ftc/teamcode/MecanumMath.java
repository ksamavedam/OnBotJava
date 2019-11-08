package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MecanumMath {
    double radius=0;
    double radiusAngle=0;
    double wheelRadius=0;
    double length=0;
    double width=0;
    
    //if Polar pear=-1 and Grizzly pear=1
    

    public MecanumMath(RobotHardware rh) {
        width = rh.widthReturn();
        length = rh.lengthReturn();
        wheelRadius = rh.wheelReturn();  
        radiusAngle = Math.toDegrees(Math.atan((width/2)/(length/2)));
        radius = Math.sqrt(Math.pow(length, 2)+Math.pow(width, 2))/2;
    }
    public double power1(double speed, double rotationSpeed, double direction, double gears){
        double frontL;
        frontL = gears*(speed*Math.cos(direction)+ radius *rotationSpeed*Math.cos(Math.toRadians(radiusAngle+90))-((speed*Math.sin(direction)+radius*rotationSpeed*Math.sin(Math.toRadians(radiusAngle+90)))/Math.tan(Math.toRadians(45))))/wheelRadius;
        return frontL;
    }
    public double power2(double speed, double rotationSpeed, double direction, double gears){
        double frontR;
        frontR = gears*(speed*Math.cos(direction)+ radius *rotationSpeed*Math.cos(Math.toRadians(270-radiusAngle))-((speed*Math.sin(direction)+radius*rotationSpeed*Math.sin(Math.toRadians(270-radiusAngle)))/Math.tan(Math.toRadians(135))))/wheelRadius;
        return frontR;
    }
    public double power3(double speed, double rotationSpeed, double direction, double gears){
        double backR;
        backR = gears*(speed*Math.cos(direction)+ radius *rotationSpeed*Math.cos(Math.toRadians(radiusAngle+270))-((speed*Math.sin(direction)+radius*rotationSpeed*Math.sin(Math.toRadians(radiusAngle+270)))/Math.tan(Math.toRadians(45))))/wheelRadius;
        return backR;
    }
    public double power4(double speed, double rotationSpeed, double direction, double gears){
        double backL;
        backL = gears*(speed*Math.cos(direction)+ radius *rotationSpeed*Math.cos(Math.toRadians(90-radiusAngle))-((speed*Math.sin(direction)+radius*rotationSpeed*Math.sin(Math.toRadians(90-radiusAngle)))/Math.tan(Math.toRadians(135))))/wheelRadius;
        return backL;
    }
}
