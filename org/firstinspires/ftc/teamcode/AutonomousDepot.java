package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import java.util.Arrays;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;
import java.lang.*;

@Autonomous(name="AutonomousDepot", group="Linear Opmode")

public class AutonomousDepot extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AQapiaj/////AAABmR9MFwGXtUXlokDNoVqBfPgVJQUtQjEGM5ThSHmsuy4picaSUk8W+xn1vM+EV1DbJfrr58EOVEJdMfLFvG4An8oN8YDvHB44IGFPAmQBdHv3RkhbYEgWU/guwcEjwIXtcRTRt/J0PmTZG2xnyDxFfAk+AOUVtLE/Ze481z/We0oTolHGpStuPrUhGKGQcY7noVFb2q2LU/3DRoUKg7R1P7y93lluKthHr2BXZSFuHqN4CmEzXdeJKQ+vZrJxorguqiIdyiNvJ1fjXAtCATQeLAOwGQWp7HlRb/T9UUf/KXLcYxKKQV3NPtvE3iPuRkDceF3IvbPbX3i32+MKXZqKFHhpZwomR9PpqEzaxgG8pJ1I";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    RobotHardware rh;
    MecanumMath mwm;
    @Override
    public void runOpMode(){
        rh = new RobotHardware("Robot2", hardwareMap);
        mwm = new MecanumMath(rh);
        rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //rh.latchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rh.latchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            int encoders;
            double Power1;
            double Power2;
            double Power3;
            double Power4;
            double position = 5;
            //Lower Robot
            //lowerPear();
            shakeyshakey();
            //Move to gold detection
            //Detect Gold Mineral
            while (opModeIsActive() && position == 5) {
                position = sampling();
            }
            //Run over gold mineral
            //Left gold
            if (position == -1) {
                encoders = horizontalEncoder(16);
                move(264, encoders, 0);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                encoders = verticalEncoder(16);
                move(0, encoders, 0);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                encoders = horizontalEncoder(24);
                move(268, encoders, 0);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                encoders = rotationEncoder(23);
                rotate(encoders);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                encoders = horizontalEncoder(18);
                move(264, encoders, 0);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //claimDepot();
                encoders = rotationEncoder(-65);
                rotate(encoders);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                encoders = rotationEncoder(24);
                rotate(encoders);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                encoders = verticalEncoder(5);
                move(0, encoders, 0);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                /*rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
            }
            //Center gold
            if (position == 0) {
                encoders = horizontalEncoder(55);
                move(264, encoders, 0);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //claimDepot();
                encoders = rotationEncoder(-50);
                rotate(encoders);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                encoders = rotationEncoder(-45);
                rotate(encoders);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                encoders = verticalEncoder(5);
                move(180, encoders, 0);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                /*rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
            }
            //Right gold
            else {
                encoders = horizontalEncoder(16);
                move(264, encoders, 0);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                encoders = verticalEncoder(15);
                move(180, encoders, 0);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                encoders = horizontalEncoder(33);
                move(264, encoders, 0);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                encoders = rotationEncoder(-21.5);
                rotate(encoders);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                encoders = horizontalEncoder(20);
                move(264, encoders, 0);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                encoders = rotationEncoder(-35);
                rotate(encoders);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //claimDepot();
                encoders = verticalEncoder(3);
                move(180, encoders, 0);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                encoders = horizontalEncoder(25);
                move(264, encoders, 0);
                rh.motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rh.motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                /*rh.motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rh.motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
            }
            //Move towards crater
            //Extend arm into crater
            //extendFull();
            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
    /*public void lowerPear(){
        rh.latchMotor.setTargetPosition(-9475);
        rh.latchMotor.setPower(-1);
        while (opModeIsActive() && rh.latchMotor.isBusy()){
            telemetry.addData("Encoders", rh.latchMotor.getCurrentPosition());
            telemetry.update();
        }
        rh.latchMotor.setPower(0);
    }*/
    public void shakeyshakey(){
        int encoders = verticalEncoder(1);
        move(0, encoders, 0);
        encoders = verticalEncoder(1);
        move(180, encoders, 0);
    }
    public int sampling(){
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        int position = -1;
        if (opModeIsActive()) {
            if (tfod != null) {
                tfod.activate();
            }
        
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        Recognition lowest = null;
                        Recognition secondLowest = null;
                        for (Recognition recognition : updatedRecognitions) {
                            if (lowest == null || recognition.getLeft() < lowest.getLeft()) {
                                secondLowest = lowest;
                                lowest = recognition;
                            } else if (secondLowest == null || recognition.getLeft() < secondLowest.getLeft()) {
                                secondLowest = recognition;
                            }
                        }
                        if (secondLowest == null) {
                            continue;
                        }
                        List<Recognition> lowests = Arrays.asList(lowest, secondLowest);
                        for (Recognition recognition : lowests) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getBottom();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getBottom();
                            }
                        }
                        if (goldMineralX == -1) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            return 1;
                        } else if (goldMineralX > silverMineral1X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            return -1;
                        } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            return 0;
                        }
                    }
                    telemetry.update();
                }
            }
            if (tfod != null) {
                tfod.shutdown();
            }
        }
        return 5;  // will never happen
    }
    private void initVuforia() {
        //Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = rh.hwMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", rh.hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    public int verticalEncoder(double distance) {
        int encoders;
        encoders = (int) Math.round(distance*31.008);
        return encoders;
    }
    public int horizontalEncoder(double distance) {
        int encoders;
        encoders = (int) Math.round(distance*37.037);
        return encoders;
    }
    public int rotationEncoder(double degrees) {
        int encoders;
        encoders = (int) Math.round(degrees*15.152);
        return encoders;
    }
    public int diagonalEncoder(double distance) {
        int encoders;
        encoders = (int) Math.round(distance*48.781);
        return encoders;
    }
    public void move(double direction, int encoders, int diagonal) {
        double Power1 = 0;
        double Power2 = 0;
        double Power3 = 0;
        double Power4 = 0;
        direction = Math.toRadians(direction);
        if (diagonal == 0) {
            Power1 = mwm.power1(.5 , 0.0, direction, 1);
            Power2 = mwm.power2(.5 , 0.0, direction, 1);
            Power3 = mwm.power3(.5 , 0.0, direction, 1);
            Power4 = mwm.power4(.5 , 0.0, direction, 1);
        }
        else if (diagonal == 1) {
            Power1 = -mwm.power1(1.3, 0.0, direction, 1);
            Power2 = mwm.power2(1.3, 0.0, direction, 1);
            Power3 = -mwm.power3(1.3, 0.0, direction, 1);
            Power4 = mwm.power4(1.3, 0.0, direction, 1);
        }
        // Send calculated power to wheels
        rh.motor1.setPower(Power1);
        rh.motor2.setPower(Power2);
        rh.motor3.setPower(Power3);
        rh.motor4.setPower(Power4);
        if (Power1 > 0) {
            rh.motor1.setTargetPosition(encoders);
        }
        else {
            rh.motor1.setTargetPosition(-encoders);
        }
        if (Power2 > 0) {
            rh.motor2.setTargetPosition(encoders);
        }
        else {
            rh.motor2.setTargetPosition(-encoders);
        }
        if (Power3 > 0) {
            rh.motor3.setTargetPosition(encoders);
        }
        else {
            rh.motor3.setTargetPosition(-encoders);
        }
        if (Power4 > 0) {
            rh.motor4.setTargetPosition(encoders);
        }
        else {
            rh.motor4.setTargetPosition(-encoders);
        }
        while (opModeIsActive() && rh.motor1.isBusy()){
            telemetry.addData("power1", Power1);
            telemetry.addData("power2", Power2);
            telemetry.addData("power3", Power3);
            telemetry.addData("power4", Power4);
            telemetry.addData("status", "moving");
            telemetry.update();
            
        }
        rh.motor1.setPower(0);
        rh.motor2.setPower(0);
        rh.motor3.setPower(0);
        rh.motor4.setPower(0);
    }
    public void rotate(int encoders) {
        double Power1;
        double Power2;
        double Power3;
        double Power4;
        Power1 = -mwm.power1(0.0, .2, 0.0, 1);
        Power2 = mwm.power2(0.0, .2, 0.0, 1);
        Power3 = -mwm.power3(0.0, .2, 0.0, 1);
        Power4 = mwm.power4(0.0, .2, 0.0, 1);
        if (Power1 > 1) {
            Power1 = Power1/3;
            Power2 = Power2/3;
            Power3 = Power3/3;
            Power4 = Power4/3;
        }
        rh.motor1.setPower(Power1);
        rh.motor2.setPower(Power2);
        rh.motor3.setPower(Power3);
        rh.motor4.setPower(Power4);
        if (Power1 > 0) {
            rh.motor1.setTargetPosition(encoders);
        }
        else {
            rh.motor1.setTargetPosition(-encoders);
        }
        if (Power2 > 0) {
            rh.motor2.setTargetPosition(encoders);
        }
        else {
            rh.motor2.setTargetPosition(-encoders);
        }
        if (Power3 > 0) {
            rh.motor3.setTargetPosition(encoders);
        }
        else {
            rh.motor3.setTargetPosition(-encoders);
        }
        if (Power4 > 0) {
            rh.motor4.setTargetPosition(encoders);
        }
        else {
            rh.motor4.setTargetPosition(-encoders);
        }
        while (opModeIsActive() && rh.motor1.isBusy()){
            telemetry.addData("power1", Power1);
            telemetry.addData("power2", Power2);
            telemetry.addData("power3", Power3);
            telemetry.addData("power4", Power4);
            telemetry.addData("status", "rotating");
            telemetry.update();
        }
        rh.motor1.setPower(0);
        rh.motor2.setPower(0);
        rh.motor3.setPower(0);
        rh.motor4.setPower(0);
    }
    /*public void claimDepot(){
        rh.depot.setPosition(0);
        sleep(50);
        rh.depot.setPosition(1);
        sleep(50);
    }*/
    /*public void extendFull(){
        rh.minShoulder.setPower(-2/3);
        rh.minShoulder.setTargetPosition(0);
        while (opModeIsActive() && rh.minShoulder.isBusy()){
        }
        rh.minShoulder.setPower(0);
    }*/
}
