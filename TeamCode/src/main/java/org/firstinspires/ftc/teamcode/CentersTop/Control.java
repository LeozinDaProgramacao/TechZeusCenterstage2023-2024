package org.firstinspires.ftc.teamcode.CentersTop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class Control extends LinearOpMode {
    Extras ex = new Extras();
    @Override
    public void runOpMode() throws InterruptedException {
        setup();
        waitForStart();

        while(opModeIsActive()){
            runMode();
        }
    }
    public void setup(){
        ex.HardwareMapAll();
    }
    public void runMode(){
        ex.addTelemetry(telemetry);
    }
}
