package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.lang.Math;


public class RobotHardware {
    public BNO055IMU imu;
    public DcMotor tlMotor;
    public DcMotor blMotor;
    public DcMotor brMotor;
    public DcMotor trMotor;
    public DcMotor grabberMotor;

    Servo foundationServo;
    Servo grabber;
    double ticksToInchV = 32.0;
    double ticksToInchH = 37.0;
    double ticksToInchR = 15.0;
    double ticksToInchD = 49.0;
    HardwareMap hwMap = null;

    public VuforiaLocalizer vuforia;
    private static final String VUFORIA_KEY = 
                "AQapiaj/////AAABmR9MFwGXtUXlokDNoVqBfPgVJQUtQjEGM5ThSHmsuy4picaSUk8W+xn1vM+EV1DbJfrr58EOVEJdMfLFvG4An8oN8YDvHB44IGFPAmQBdHv3RkhbYEgWU/guwcEjwIXtcRTRt/J0PmTZG2xnyDxFfAk+AOUVtLE/Ze481z/We0oTolHGpStuPrUhGKGQcY7noVFb2q2LU/3DRoUKg7R1P7y93lluKthHr2BXZSFuHqN4CmEzXdeJKQ+vZrJxorguqiIdyiNvJ1fjXAtCATQeLAOwGQWp7HlRb/T9UUf/KXLcYxKKQV3NPtvE3iPuRkDceF3IvbPbX3i32+MKXZqKFHhpZwomR9PpqEzaxgG8pJ1I";
    public TFObjectDetector tfod;

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
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
        
        initTfod();
        initVuforia();
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public double getTicksToInchV() {return ticksToInchV; }
    public double getTicksToInchH() {return ticksToInchH; }
    public double getTicksToInchR() {return ticksToInchR; }
    public double getTicksToInchD() {return ticksToInchD; }

    
}
