package org.firstinspires.ftc.teamcode.Nacional.SubSystems;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class ClawColorSensor {
    public static boolean isPixelInClaw(ColorSensor Sensor){
        //if color purple, resp = true
        //if color yellow, resp  = true
        //if color green resp true
        //if color white resp true
        return Sensor.red() >= 170
                || Sensor.green() >= 230
                || Sensor.blue() >= 170;
    }
}
