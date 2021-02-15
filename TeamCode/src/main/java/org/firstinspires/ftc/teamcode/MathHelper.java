package org.firstinspires.ftc.teamcode;

public class MathHelper {

    public static double wrapAngle (double angle) {
        while (angle<-Math.PI){
            angle += 2*Math.PI;
        }
        while (angle>Math.PI){
            angle -= 2*Math.PI;
        }
        return angle;
    }
}
