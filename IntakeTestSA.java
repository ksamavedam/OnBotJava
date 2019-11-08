
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class IntakeTestSA extends LinearOpMode {
    CRServo intakeArm = null;
    Servo arm = null;
    double dampen = 0.5;
    @Override
    public void runOpMode() {
        intakeArm = hardwareMap.get(CRServo.class, "intake");
        arm = hardwareMap.get(Servo.class, "arm");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        
        while (opModeIsActive()) {
            
            if (gamepad1.a) {
                intakeArm.setPower(0.70);
        telemetry.addData("Status", "Gamepad A");
        telemetry.update();
                
            }
            if (gamepad1.b) {
                intakeArm.setPower(-0.70);
            }
            if (gamepad1.x){
                intakeArm.setPower(0);
            }
            
           if(gamepad1.dpad_up){
               arm.setPosition(0.50);
           }
           if(gamepad1.dpad_down){
               arm.setPosition(-0.90);
           }
           
            
            
            
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}

