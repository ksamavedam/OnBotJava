package org.firstinspires.ftc.teamcode;

import java.util.Arrays;
import java.lang.Math;


public class OttermelonMotion {

     public static double[] move(double angle, double scale, double turnScale){
        double topLeft=0;
        double bottomLeft=0;
        double bottomRight=0;
        double topRight= 0;
        double[] powers= new double[4];
        if(scale!=0){
            topLeft=(Math.cos(angle)) + -1*(Math.sin(angle))+turnScale*1;
            bottomLeft=(Math.cos(angle)) + (Math.sin(angle))+turnScale*1;
            bottomRight=(Math.cos(angle)) + -1*(Math.sin(angle))+turnScale*-1;
            topRight=(Math.cos(angle)) + (Math.sin(angle))+turnScale*-1;
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
            /*powers[0]=topLeft;
            powers[1]=bottomLeft;
            powers[2]=bottomRight;
            powers[3]=topRight;*/
            powers[0]=topLeft;
            powers[1]=bottomLeft;
            powers[2]=bottomRight;
            powers[3]=topRight;
        }
        else {
            powers[0]= -1*turnScale;
            powers[1]= -1*turnScale;
            powers[2]=1*turnScale;
            powers[3]=1*turnScale;
        }
        return powers;
}

    
}
