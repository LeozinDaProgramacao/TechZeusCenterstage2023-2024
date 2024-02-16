package org.firstinspires.ftc.teamcode.Nacional.SubSystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AirplaneLauncher {
    public static double LAUNCH_AIRPLANE_POSITION=1;
    public static double HOLD_AIRPLANE_POSITION=0;

    public static void resetAirplaneServo(){
        RobotHardware.setAirplanePosition(HOLD_AIRPLANE_POSITION);
    }
    public static void launchAirplane(double LTrigger,double RTrigger){
        if (LTrigger>0.9&&RTrigger>0.9) {
            RobotHardware.setAirplanePosition(LAUNCH_AIRPLANE_POSITION);
        } else {
            RobotHardware.setAirplanePosition(HOLD_AIRPLANE_POSITION);
        }
    }
}
