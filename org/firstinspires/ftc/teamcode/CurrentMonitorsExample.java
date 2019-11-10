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
import com.qualcomm.robotcore.util.ElapsedTime;
//import org.openftc.revextensions2.ExpansionHubEx;
//import org.openftc.revextensions2.ExpansionHubMotor;

@Autonomous(name="CurrentMonitors", group="Linear Opmode")

public class CurrentMonitorsExample extends LinearOpMode{
    private BNO055IMU imu;
    private DcMotor tlMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private DcMotor trMotor;
    private DcMotor grabberMotor;
    private Servo foundationServo;
    private Servo grabber;
    private Blinker expansion_Hub_2;
    ExpansionHubEx expansionHub;
    ExpansionHubMotor tlMotorEx, blMotorEx, brMotorEx, trMotorEx;
    @Override
        public void runOpMode(){
            BNO055IMU.Parameters imuParameters;
            Orientation angles;
            expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
            tlMotor=hardwareMap.get(DcMotor.class, "topLeft");
            blMotor=hardwareMap.get(DcMotor.class, "bottomLeft");
            brMotor=hardwareMap.get(DcMotor.class, "bottomRight");
            trMotor=hardwareMap.get(DcMotor.class, "topRight");

            tlMotorEx = (ExpansionHubMotor)tlMotor ;
            blMotorEx = (ExpansionHubMotor)blMotor ;
            trMotorEx = (ExpansionHubMotor)trMotor ;
            brMotorEx = (ExpansionHubMotor)brMotor ;


            tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

 /*           
            tlMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  getCurrentPosition()
            blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            trMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/
            tlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            trMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            
            
            tlMotor.setDirection(DcMotor.Direction.REVERSE);
            blMotor.setDirection(DcMotor.Direction.REVERSE);
            
            ElapsedTime elp_time = new ElapsedTime();
            double start_ms = 0, end_ms = 0, rpm = 0;
            int start_flag = 0, start_enc = 0, end_enc = 0; 
            waitForStart();
            while(opModeIsActive()){
            if ( elp_time.milliseconds() > 5000.0 && start_flag == 0 ) {
                    start_flag = 1; 
                    start_ms = elp_time.milliseconds();
                    start_enc = tlMotor.getCurrentPosition(); 
            }
            if ( elp_time.milliseconds() > 10000.0 && start_flag == 1 ) {
                start_flag = 1; 
                end_ms = elp_time.milliseconds();
                end_enc = tlMotor.getCurrentPosition(); 
                rpm = ((end_enc - start_enc)/537.6)*60*1000/(end_ms - start_ms);
            }
            double p = 0.1;
            tlMotor.setPower(p);
            blMotor.setPower(p);
            brMotor.setPower(p);
            trMotor.setPower(p);

            //telemetry.addData("curr pos ", String.format("%d", tlMotor.getCurrentPosition()));
            //telemetry.addData("curr elp ", String.format("%f", elp_time.milliseconds()));
            telemetry.addData("curr pos ", String.format("%d\t%d\t%d", tlMotor.getCurrentPosition(), start_enc, end_enc));
            telemetry.addData("curr elp ", String.format("%f\t%f\t%f \t %f", elp_time.milliseconds(), start_ms/1000.0, end_ms/1000.2, rpm));
            telemetry.update();
            displayCurrent();
            
            /*tlMotor.setPower(0);
            blMotor.setPower(0);    
            brMotor.setPower(0);
            trMotor.setPower(0);*/
            
            }
            tlMotor.setPower(0);
            blMotor.setPower(0);
            brMotor.setPower(0);
            trMotor.setPower(0);            
        }

        public void displayCurrent() {
            String header =
            "**********************************\n" +
            "CURRENT MONITORING EXAMPLE        \n" +
            "NOTE: UNITS ARE AMPS              \n" +
            "**********************************\n";
            telemetry.addLine(header);

            telemetry.addData("Total current", expansionHub.getTotalModuleCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.addData("I2C current", expansionHub.getI2cBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.addData("GPIO current", expansionHub.getGpioBusCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.addData("M0 current", tlMotorEx.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.addData("M1 current", blMotorEx.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.addData("M2 current", brMotorEx.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
            telemetry.addData("M3 current", trMotorEx.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));

            telemetry.update();
        }
    }
        
