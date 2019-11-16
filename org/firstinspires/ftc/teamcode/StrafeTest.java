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
import org.firstinspires.ftc.teamcode.OttermelonNavigation;
import org.firstinspires.ftc.teamcode.OttermelonMotion;

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

            tlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
            blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
            brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
            trMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

            
            tlMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            trMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            /*tlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            trMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
            
            tlMotor.setDirection(DcMotor.Direction.REVERSE);
            blMotor.setDirection(DcMotor.Direction.REVERSE);
            
            //OttermelonNavigation robot= new OttermelonNavigation();
            waitForStart();
            while(opModeIsActive()){
            
             int targetPosition= 1000;
            /*tlMotor.setTargetPosition(targetPosition);
            blMotor.setTargetPosition(targetPosition);
            brMotor.setTargetPosition(targetPosition);
            trMotor.setTargetPosition(targetPosition);*/
           //robot.moveDist(10.0, 0.0, .5);
           moveDist(30.0,180.0,0.5);
           moveDist(30.0,0,.5);
           moveDist(25, 180.0, .5);
           moveDist(40, 90.0, .5);
           
            
           break;
            }
        }

            public void moveDist(double dist, double angle, double power){

                
                double f_Ticks_Per_Inch=57.14;
                double s_Ticks_Per_Inch=62.5;
                double targetPosition=0;
                
                if(angle==0){
        
                  targetPosition=f_Ticks_Per_Inch*dist;
                  
                }
                else if (angle==180) {
                  targetPosition=f_Ticks_Per_Inch*dist;
                  
                }
                else if (angle==90) {
                  targetPosition=s_Ticks_Per_Inch*dist;
                  
                }
                else if (angle==270) {
                  targetPosition=s_Ticks_Per_Inch*dist;
                  
                }
                else {
        
                  throw new IllegalArgumentException("Ya can't be enterin an angle other than 0, 90 ,180 or 270.");
                }
                angle = Math.toRadians(angle);
                moveAngle(angle, power);
                while(Math.abs(tlMotor.getCurrentPosition())<targetPosition){
        
                    
                }
                 setPower(0, 0, 0, 0);
                 reset();
              }
        
              public void setPower(double tlPower, double blPower, double brPower, double trPower){
        
                tlMotor.setPower(tlPower);
                blMotor.setPower(blPower);
                brMotor.setPower(brPower);
                trMotor.setPower(trPower);
              }

              public void moveAngle(double angle, double power){

                double[] powers= new double[4];
                powers= OttermelonMotion.move(angle, power, 0);
                tlMotor.setPower(powers[0]);
                blMotor.setPower(powers[1]);
                brMotor.setPower(powers[2]);
                trMotor.setPower(powers[3]);
              }

              public void reset(){

                tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                tlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
                trMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
              }
        }
    
        
