package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class RobotHardware {
    public BNO055IMU imu;
    public DcMotor tlMotor=null;
    public DcMotor blMotor=null;
    public DcMotor brMotor=null;
    public DcMotor trMotor=null;
    public DcMotor intakeMotorLeft=null;
    public DcMotor intakeMotorRight=null;
    public DcMotor slideRight;
    public DcMotor slideLeft;
    public DistanceSensor sensorRange=null;
    public Servo gripper;
    public Servo armRight;
    public Servo armLeft;
    public Servo level;
    double ticksToInchV = 32.0;
    double ticksToInchH = 37.0;
    double ticksToInchR = 15.0;
    double ticksToInchD = 49.0;
    HardwareMap hwMap = null;
    public TFObjectDetector tfod;
    public VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY = 
    "AQapiaj/////AAABmR9MFwGXtUXlokDNoVqBfPgVJQUtQjEGM5ThSHmsuy4picaSUk8W+xn1vM+EV1DbJfrr58EOVEJdMfLFvG4An8oN8YDvHB44IGFPAmQBdHv3RkhbYEgWU/guwcEjwIXtcRTRt/J0PmTZG2xnyDxFfAk+AOUVtLE/Ze481z/We0oTolHGpStuPrUhGKGQcY7noVFb2q2LU/3DRoUKg7R1P7y93lluKthHr2BXZSFuHqN4CmEzXdeJKQ+vZrJxorguqiIdyiNvJ1fjXAtCATQeLAOwGQWp7HlRb/T9UUf/KXLcYxKKQV3NPtvE3iPuRkDceF3IvbPbX3i32+MKXZqKFHhpZwomR9PpqEzaxgG8pJ1I";
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";  

    public RobotHardware(String name, HardwareMap hw) {


        if (name == "OtterMelon") {
            ticksToInchV = 62.5;
            ticksToInchH = 57.2;
            ticksToInchR = 15.0;
            ticksToInchD = 49.0;
        } else if (name == "Ri3D") {
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
        gripper=hwMap.get(Servo.class, "gripper");
        armLeft=hwMap.get(Servo.class, "armLeft");
        armRight=hwMap.get(Servo.class, "armRight");
        level=hwMap.get(Servo.class, "level");
        
        

        imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
        
        tlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (name == "OtterMelon") {
            intakeMotorLeft=hwMap.get(DcMotor.class, "leftIntake");
            intakeMotorRight=hwMap.get(DcMotor.class, "rightIntake");
            sensorRange = hwMap.get(DistanceSensor.class, "distanceS"); 
            slideLeft=hwMap.get(DcMotor.class, "slideLeft");
            slideRight=hwMap.get(DcMotor.class, "slideRight");   
           
            // telemetry.addData("%s", "  ITS A SKYSTONE");
            tlMotor.setDirection(DcMotor.Direction.REVERSE);
            brMotor.setDirection(DcMotor.Direction.REVERSE);

        }
        
        else if (name == "AppleBee") {
                tlMotor.setDirection(DcMotor.Direction.REVERSE);
                brMotor.setDirection(DcMotor.Direction.REVERSE);
        } 
    }
    
    public double getTicksToInchV() {return ticksToInchV; }
    public double getTicksToInchH() {return ticksToInchH; }
    public double getTicksToInchR() {return ticksToInchR; }
    public double getTicksToInchD() {return ticksToInchD; }

    public void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id",
        hwMap.appContext.getPackageName());
       //TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraDirection = CameraDirection.BACK;
        parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
}
