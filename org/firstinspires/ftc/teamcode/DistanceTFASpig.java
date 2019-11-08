package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Blinker;
import java.util.Arrays;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;



// KS



@TeleOp(name="DistanceTFASpipg", group="Linear Opmode")



public class DistanceTFASpig extends LinearOpMode{
    
    private Gyroscope imu;
    private DcMotor tlMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;
    private DcMotor trMotor;
    private Blinker expansion_Hub_2;
    private DistanceSensor sensorRange;
    double distance= 0;
    
    @Override
    public void runOpMode(){
        
        tlMotor=hardwareMap.get(DcMotor.class, "topLeft");
        blMotor=hardwareMap.get(DcMotor.class, "bottomLeft");
        brMotor=hardwareMap.get(DcMotor.class, "bottomRight");
        trMotor=hardwareMap.get(DcMotor.class, "topRight");
        sensorRange = hardwareMap.get(DistanceSensor.class, "distanceS");
        
         double[] setpowers= new double[4];
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        
        tlMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);
        
                tlMotor.setTargetPosition(0);
                blMotor.setTargetPosition(0);
                brMotor.setTargetPosition(0);
                trMotor.setTargetPosition(0);
                
        tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
        
        tlMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        trMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS); 
        
        waitForStart();
        
      
        while(opModeIsActive()){
            //resetEncoders();
                distance= sensorRange.getDistance(DistanceUnit.INCH);
               // moveEnc(180,0.05,0,(distance-3));
         
                //move(direction, speed, turning speed)
                setpowers=move(180, 0.25, 0);
                /*tlMotor.setPower(1);
                blMotor.setPower(1);
                brMotor.setPower(1);
                trMotor.setPower(1); */

                tlMotor.setPower(setpowers[0]);
                blMotor.setPower(setpowers[1]);
                brMotor.setPower(setpowers[2]);
                trMotor.setPower(setpowers[3]); 

           do {
                distance= sensorRange.getDistance(DistanceUnit.INCH);
                
                telemetry.addData("range", String.format("%.01f in", distance));
                telemetry.addData("Test",0);
                telemetry.addData("TL", setpowers[0]);
                telemetry.addData("BL", setpowers[1]);
                telemetry.addData("BR", setpowers[2]);
                telemetry.addData("TR", setpowers[3]);

                telemetry.update();
            } while(distance>=3);
            
                tlMotor.setPower(0);
                blMotor.setPower(0);
                brMotor.setPower(0);
                trMotor.setPower(0);
                break; 
               
           } 
            tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); 
            
        } 

    
    public void moveEnc(double angle, double scale, double turnScale, double distEnc){
        double r_angle=Math.toRadians(angle);
        double ticksToInchV=32.0;
        double ticksToInchH=37.0;
        double ticksToInchR=15.0;
        double ticksToInchD=49.0;

        double[] powers= move(r_angle, scale, turnScale);
        double encoders;

        if(scale != 0) {
            if(angle==0){
                encoders = distEnc*ticksToInchV;
                tlMotor.setTargetPosition((int)encoders);
                blMotor.setTargetPosition((int)encoders);
                brMotor.setTargetPosition((int)encoders);
                trMotor.setTargetPosition((int)encoders);
            }
            else if(angle==180){
                encoders = distEnc*ticksToInchV;
                tlMotor.setTargetPosition(-(int)encoders);
                blMotor.setTargetPosition(-(int)encoders);
                brMotor.setTargetPosition(-(int)encoders);
                trMotor.setTargetPosition(-(int)encoders);
            }
            else if(angle==90){
                encoders=distEnc+ticksToInchH;
                tlMotor.setTargetPosition((int)encoders*-1);
                blMotor.setTargetPosition((int)encoders);
                brMotor.setTargetPosition((int)encoders*-1);
                trMotor.setTargetPosition((int)encoders);
            }
            else{
                encoders = distEnc*ticksToInchH;
                tlMotor.setTargetPosition((int)encoders);
                blMotor.setTargetPosition((int)encoders*-1);
                brMotor.setTargetPosition((int)encoders);
                trMotor.setTargetPosition((int)encoders*-1);
            }
        }
        else {
            encoders = distEnc*ticksToInchR;
            
            if(turnScale < 0) {
                tlMotor.setTargetPosition((int)encoders);
                blMotor.setTargetPosition((int)encoders);
                brMotor.setTargetPosition(-1 * (int)encoders);
                trMotor.setTargetPosition(-1 * (int)encoders);
            }
            else {
                tlMotor.setTargetPosition(-1 * (int)encoders);
                blMotor.setTargetPosition(-1 * (int)encoders);
                brMotor.setTargetPosition((int)encoders);
                trMotor.setTargetPosition((int)encoders);
            }

        }

        tlMotor.setPower(powers[0]);
        blMotor.setPower(powers[1]);
        brMotor.setPower(powers[2]);
        trMotor.setPower(powers[3]);
        
       /* telemetry.addData("Test",0);
        telemetry.addData("TL", powers[0]);
        telemetry.addData("BL", powers[1]);
        telemetry.addData("BR", powers[2]);
        telemetry.addData("TR", powers[3]);*/
        
        
    }
    
    public void resetEncoders(){
        tlMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        trMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
       /* tlMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        trMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
    }
   
    
    public double[] move(double angle, double scale, double turnScale){
        double topLeft=0;
        double bottomLeft=0;
        double bottomRight=0;
        double topRight= 0;
        double[] powers= new double[4];
        
        if(scale!=0.0){
            topLeft=(Math.cos(angle)/Math.sqrt(2)) + -1*(Math.sin(angle))+(turnScale*1);
            bottomLeft=(Math.cos(angle)/Math.sqrt(2)) + (Math.sin(angle))+(turnScale*1);
            bottomRight=(Math.cos(angle)/Math.sqrt(2)) + -1*(Math.sin(angle))+(turnScale*-1);
            topRight= (Math.cos(angle)/Math.sqrt(2)) + (Math.sin(angle))+(turnScale*-1);
            powers[0]=topLeft;
            powers[1]=bottomLeft;
            powers[2]=bottomRight;
            powers[3]=topRight;
            
            for(int i=0; i<powers.length; i++){
                powers[i]=Math.abs(powers[i]);
            }
            Arrays.sort(powers);
            topLeft=topLeft/powers[3];
            bottomLeft=bottomLeft/powers[3];
            bottomRight=bottomRight/powers[3];
            topRight=topRight/powers[3];

            topLeft*=scale;
            bottomLeft*=scale;
            bottomRight*=scale;
            topRight*=scale;   
            powers[0]=topLeft;
            powers[1]=bottomLeft;
            powers[2]=bottomRight;
            powers[3]=topRight;
        }
        else {
            powers[0] = 1*turnScale;
            powers[1] = 1*turnScale;
            powers[2] = -1*turnScale;
            powers[3] = -1*turnScale;
            
        }
        return powers;
    }
    
    
}




   

      
        
          
 
