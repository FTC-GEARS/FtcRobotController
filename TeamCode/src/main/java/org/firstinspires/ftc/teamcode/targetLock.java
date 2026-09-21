package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

//automaticlly aims at hives using april tags and known field position

public class targetLock {

    public void initTargetLock(HardwareMap hardwareMap){

    }

    public double getLockAngle(double x, double y, double targetX, double targetY, double width){
        //one unit of x or y is one CM
        double a = targetX - x;
        double b = targetY - y;
        double theta = 0;

        theta = Math.atan(b/a);

        //correct swapping
        if (y > width/2) {
            theta *= -1;
        }
        //----------------------------------------------------
        //PUT APRIL TAG ALLIGNMENT HERE
        //----------------------------------------------------

        return theta;

    }

}
