package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class mainGuts {

    private static DcMotorEx shooter, passer;

    public void initGuts(HardwareMap hardwareMap){

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        passer = hardwareMap.get(DcMotorEx.class, "passer");

    }

}
