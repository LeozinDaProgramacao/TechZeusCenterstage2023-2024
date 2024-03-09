package org.firstinspires.ftc.teamcode.Nacional.SubSystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Nacional.Utility.PID;
@Config
public class HangRobot {
    private static final double HANGVARIATION = 16;
    public static double HOOK_HEIGHT_GRAB=700;
    public static double HOOK_HEIGHT_HANG=130;
    public static double HOOK_HEIGHT_STORE=0;
    public static double currentDesiredHeight =0;
    static PID lftHexPIDf = new PID(0.03,0.0,0,0);
    static PID rgtHexPIDf = new PID(0.03,0,0,0);
    public static double goalhang;


    public static void extendHooksToGrab(){
        currentDesiredHeight = HOOK_HEIGHT_GRAB;
        //pidf set goal to HOOK_EIGHT_GRAB

    }
    public static void pullHooksToHang(){
        currentDesiredHeight = HOOK_HEIGHT_HANG;
    }
    public static void hideHooks(){
        currentDesiredHeight = HOOK_HEIGHT_STORE;
    }
    public static void HangLoop(boolean raisehooks,boolean hangnow){
        if(raisehooks){
            extendHooksToGrab();
        } else if (hangnow){
            pullHooksToHang();
        }
        if (((goalhang > currentDesiredHeight - HANGVARIATION*2) && (goalhang < currentDesiredHeight + HANGVARIATION*2))) {
            goalhang = currentDesiredHeight;
        } else if (goalhang>currentDesiredHeight){
            goalhang-=HANGVARIATION;
        } else if (goalhang<currentDesiredHeight){
            goalhang+=HANGVARIATION;
        }
        if (currentDesiredHeight == HOOK_HEIGHT_GRAB){
            RobotHardware.CoreEsq.setPower(lftHexPIDf.CalculatePID(RobotHardware.CoreEsq.getCurrentPosition(),goalhang,false)-0.06);
            RobotHardware.CoreDir.setPower(rgtHexPIDf.CalculatePID(RobotHardware.CoreDir.getCurrentPosition(),goalhang,false)-0.06);
        } else{
            RobotHardware.CoreEsq.setPower(lftHexPIDf.CalculatePID(RobotHardware.CoreEsq.getCurrentPosition(),goalhang,false));
            RobotHardware.CoreDir.setPower(rgtHexPIDf.CalculatePID(RobotHardware.CoreDir.getCurrentPosition(),goalhang,false));

        }
    }
    public double getCurrentDesiredHeight(){
        return currentDesiredHeight;
    }
}
