package org.firstinspires.ftc.teamcode.Nacional.SubSystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Nacional.Utility.PID;


@Config
public class ArmMovement {
    public static enum ARM_STATE {
        DEPOSIT_FRONT,
        DEPOSIT_BACK,
        DEPOSIT_BACK_AUTO,
        HANGING,
        STORED,
        PIXEL5UP,
        PIXEL4UP,
        PIXEL3UP,
        PIXEL2UP,
        PIXEL1UP,
        DEPOSIT_NO_ARM
    }
    public static ARM_STATE currentArmState;
    public enum CLAW_STATE{
        OPEN,
        CLOSED
    }
    public static int Artheight =0;
    public static double LEFT_ARTICULATION_DIFFERENCE=-0.05;
    public static double ARTICULATION_MIDDLE=0.53;
    public static double ARTICULATION_POS_DEPOSIT_FRONT=-0.07;
    public static double ARTICULATION_POS_DEPOSIT_BACK=-0.05;
    public static double ARTICULATION_POS_DEPOSIT_BACK_AUTO = -0.05;
    public static double ARTICULATION_POS_COLLECT_GROUND=-0.03;
    public static double ARTICULATION_STORE_CLAW=-0.3;
    public static double ARTICULATION_HANG=-0.03;
    public static double ARTICULATION_POS_2NDPIXEL_UP=-0.06;
    public static double ARTICULATION_POS_3NDPIXEL_UP=-0.08;
    public static double ARTICULATION_POS_4NDPIXEL_UP=-0.09;
    public static double ARTICULATION_POS_5NDPIXEL_UP=-0.11;
    public static double ARTICULATION_DEPOSIT_NO_ARM = 0;


    public static double WRIST_POS_DEPOSIT_FRONT=-0.09;
    public static double WRIST_POS_DEPOSIT_BACK=-0.3;
    public static double WRIST_POS_DEPOSIT_BACK_AUTO=-0.3;
    public static double WRIST_POS_COLLECT_GROUND=-0.05;
    public static double WRIST_STORE_CLAW=-0.3;
    public static double WRIST_MIDDLE=0.4;
    public static double WRIST_HANG;
    public static double WRIST_POS_2NDPIXEL_UP=-0.05;
    public static double WRIST_POS_3NDPIXEL_UP=-0.05;
    public static double WRIST_POS_4NDPIXEL_UP=-0.04;
    public static double WRIST_POS_5NDPIXEL_UP=-0.03;
    public static double WRIST_DEPOSIT_NO_ARM =0;

    public static double OPEN_CLAW_POS=0.6;
    public static double CLOSE_CLAW_POS=0.4;


    public static double ARM_DEAD_ZONE;
    public static double ARM_DOWN=0;
    public static double ARM_DEPOSIT_FRONT=500;
    public static double ARM_DEPOSIT_BACK=1600;
    public static double ARM_DEPOSIT_BACK_AUTO = 1600;
    public static double PVARIATION=0.02;
    public static int LVARIATION = 100;
    public static double finalArmGoal=0;

    public static double currentArmGoal=0;
    public static double armPower=0;
    static boolean effectiveLeft;
    static boolean effectiveRight;
    static PID armPID = new PID(0.003,0,0,0);

    public static void setArmState(ARM_STATE desiredState){
        currentArmState = desiredState;
        switch (desiredState){
            case DEPOSIT_FRONT:
                finalArmGoal = ARM_DEPOSIT_FRONT+ARM_DEAD_ZONE;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE+ARTICULATION_POS_DEPOSIT_FRONT);
                RobotHardware.setWristPos(WRIST_MIDDLE+WRIST_POS_DEPOSIT_FRONT);
                break;
            case DEPOSIT_BACK:
                finalArmGoal = ARM_DEPOSIT_BACK+ARM_DEAD_ZONE;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE+ARTICULATION_POS_DEPOSIT_BACK);
                RobotHardware.setWristPos(WRIST_MIDDLE+WRIST_POS_DEPOSIT_BACK);
                break;
            case DEPOSIT_BACK_AUTO:
                finalArmGoal = ARM_DEPOSIT_BACK_AUTO;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE+ARTICULATION_POS_DEPOSIT_BACK_AUTO);
                RobotHardware.setWristPos(ARTICULATION_MIDDLE+WRIST_POS_DEPOSIT_BACK_AUTO);
                break;
            case STORED:
                finalArmGoal=ARM_DOWN;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE+ARTICULATION_STORE_CLAW);
                RobotHardware.setWristPos(WRIST_MIDDLE+WRIST_STORE_CLAW);
                break;
            case HANGING:
                finalArmGoal=ARM_DOWN;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE+ARTICULATION_HANG);
                RobotHardware.setWristPos(WRIST_MIDDLE+WRIST_HANG);
            case PIXEL5UP:
                finalArmGoal=ARM_DOWN;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE+ARTICULATION_POS_5NDPIXEL_UP);
                RobotHardware.setWristPos(WRIST_MIDDLE+WRIST_POS_5NDPIXEL_UP);
                break;
            case PIXEL4UP:
                finalArmGoal=ARM_DOWN;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE+ARTICULATION_POS_4NDPIXEL_UP);
                RobotHardware.setWristPos(WRIST_MIDDLE+WRIST_POS_4NDPIXEL_UP);
                break;
            case PIXEL3UP:
                finalArmGoal=ARM_DOWN;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE+ARTICULATION_POS_3NDPIXEL_UP);
                RobotHardware.setWristPos(WRIST_MIDDLE+WRIST_POS_3NDPIXEL_UP);
                break;
            case PIXEL2UP:
                finalArmGoal=ARM_DOWN;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE+ARTICULATION_POS_2NDPIXEL_UP);
                RobotHardware.setWristPos(WRIST_MIDDLE+WRIST_POS_2NDPIXEL_UP);
                break;
            case PIXEL1UP:
                finalArmGoal=ARM_DOWN;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE+ARTICULATION_POS_COLLECT_GROUND);
                RobotHardware.setWristPos(WRIST_MIDDLE+WRIST_POS_COLLECT_GROUND);
                break;
            case DEPOSIT_NO_ARM:
                finalArmGoal=ARM_DOWN;
                RobotHardware.setArtPosition(0);
                RobotHardware.setWristPos(0);
                break;

        }

    }
    public static void ControlClawTeleop(boolean leftState,boolean rightState){

        if(RobotHardware.mainArm.getCurrentPosition()<1000){
            effectiveRight = rightState;
            effectiveLeft = leftState;
        } else {
            effectiveRight = leftState;
            effectiveLeft = rightState;
        }


        //effective left means the left the driver sees, because when the claw is in the up position
        // the left and right are flipped, which generates confusion for drivers
        //so if the arm is in the back position the left and right claws are flipped
        if (effectiveLeft){
            RobotHardware.LClaw.setPosition(CLOSE_CLAW_POS);
        } else {
            RobotHardware.LClaw.setPosition(OPEN_CLAW_POS);
        }
        if (effectiveRight){
            RobotHardware.RClaw.setPosition(1-CLOSE_CLAW_POS);
        } else {
            RobotHardware.RClaw.setPosition(1-OPEN_CLAW_POS);
        }
    }
    public static void ControlLeftClaw(CLAW_STATE claw_state){
        switch (claw_state){
            case OPEN:
                RobotHardware.LClaw.setPosition(OPEN_CLAW_POS);
                break;
            case CLOSED:
                RobotHardware.LClaw.setPosition(CLOSE_CLAW_POS);
                break;
        }
    }
    public static void ControlRightClaw(CLAW_STATE claw_state){
        switch (claw_state){
            case OPEN:
                RobotHardware.RClaw.setPosition(1-OPEN_CLAW_POS);
                break;
            case CLOSED:
                RobotHardware.RClaw.setPosition(1-CLOSE_CLAW_POS);
                break;
        }
    }
    public static void armPIDLoop(boolean ARM_STOP_REQUESTED){

        currentArmGoal+= PVARIATION*(finalArmGoal-currentArmGoal);
        if (((currentArmGoal > finalArmGoal - LVARIATION) && (currentArmGoal < finalArmGoal + LVARIATION))) {
            currentArmGoal = finalArmGoal;
        } else {
            currentArmGoal+= PVARIATION*(finalArmGoal-currentArmGoal);
        }
        armPower= armPID.CalculatePID(RobotHardware.mainArm.getCurrentPosition(), currentArmGoal, false);
        if (ARM_STOP_REQUESTED) {
            RobotHardware.moveArm(-0.1);
            finalArmGoal = RobotHardware.mainArm.getCurrentPosition();
            currentArmGoal = finalArmGoal;
        } else {
            RobotHardware.moveArm(armPower);
        }
    }
}
