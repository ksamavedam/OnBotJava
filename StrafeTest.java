import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Blinker;
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
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="StrafeTest", group="Linear Opmode")

public class StrafeTest extends LinearOpMode{
    private BNO055IMU imu;
    private DcMotor tlMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private DcMotor trMotor;
    private DcMotor grabberMotor;
    private Servo foundationServo;
    private Servo grabber;
    private Blinker expansion_Hub_2;
    @Override
        public void runOpMode(){
            BNO055IMU.Parameters imuParameters;
            Orientation angles;
            
            tlMotor=hardwareMap.get(DcMotor.class, "topLeft");
            blMotor=hardwareMap.get(DcMotor.class, "bottomLeft");
            brMotor=hardwareMap.get(DcMotor.class, "bottomRight");
            trMotor=hardwareMap.get(DcMotor.class, "topRight");
            
            tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            
            tlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            trMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            tlMotor.setDirection(DcMotor.Direction.REVERSE);
            blMotor.setDirection(DcMotor.Direction.REVERSE);
            
            
            waitForStart();
            //while(opModeIsActive()){
            
            tlMotor.setTargetPosition(10000);
            blMotor.setTargetPosition(10000);
            brMotor.setTargetPosition(10000);
            trMotor.setTargetPosition(10000);
            
            tlMotor.setPower(.5);
            blMotor.setPower(.5);
            brMotor.setPower(.5);
            trMotor.setPower(.5);
            
            /*tlMotor.setPower(0);
            blMotor.setPower(0);
            brMotor.setPower(0);
            trMotor.setPower(0);*/
            
            //}
        }
    }
        
