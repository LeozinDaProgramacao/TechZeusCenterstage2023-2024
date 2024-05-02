package org.firstinspires.ftc.teamcode.Nacional.SubSystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Nacional.Utility.PID;
import org.firstinspires.ftc.teamcode.Nacional.Utility.simpleSwitch;

import java.util.EnumSet;
import java.util.Set;


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
        CORRECT1UP,
        CORRECT2UP,
        CORRECT3UP,
        DEPOSIT_NO_ARM,
        DEBUG
    }
    public static enum CLAW_MODE{
        MANUAL,
        SENSOR
    }
    public static ARM_STATE currentArmState;
    public static Set<ARM_STATE> COLLECT_ARM_STATES = EnumSet.of( ARM_STATE.PIXEL5UP,
            ARM_STATE.PIXEL4UP,
            ARM_STATE.PIXEL3UP,
            ARM_STATE.PIXEL2UP,
            ARM_STATE.PIXEL1UP,
            ARM_STATE.DEPOSIT_NO_ARM );
    public enum CLAW_STATE{
        OPEN,
        CLOSED
    }
    public static int Artheight =0;
    public static double LEFT_ARTICULATION_DIFFERENCE=-0.02;
    public static double ARTICULATION_MIDDLE=0.57;
    public static double ARTICULATION_POS_DEPOSIT_FRONT=-0.07;
    public static double ARTICULATION_POS_DEPOSIT_BACK=-0.03;
    public static double ARTICULATION_POS_DEPOSIT_BACK_AUTO = -0.05;
    public static double ARTICULATION_POS_COLLECT_GROUND=-0.03;
    public static double ARTICULATION_STORE_CLAW=-0.4;
    public static double ARTICULATION_HANG=-0.03;
    public static double ARTICULATION_POS_CORRECT_1ND =-0.33;
    public static double ARTICULATION_POS_CORRECT_2ND =-0.33;
    public static double ARTICULATION_POS_CORRECT_3ND =-0.33;
    public static double ARTICULATION_POS_2NDPIXEL_UP=-0.06;
    public static double ARTICULATION_POS_3NDPIXEL_UP=-0.08;
    public static double ARTICULATION_POS_4NDPIXEL_UP=-0.11;
    public static double ARTICULATION_POS_5NDPIXEL_UP=-0.12;
    public static double ARTICULATION_DEPOSIT_NO_ARM = -0.4;


    public static double WRIST_POS_DEPOSIT_FRONT=-0.09;
    public static double WRIST_POS_DEPOSIT_BACK=-0.25;
    public static double WRIST_POS_DEPOSIT_BACK_AUTO=-0.25;
    public static double WRIST_POS_COLLECT_GROUND=-0.04;
    public static double WRIST_STORE_CLAW=-0.25;
    public static double WRIST_MIDDLE=0.45;
    public static double WRIST_HANG;
    public static double WRIST_POS_CORRECT_1ND =0.13;
    public static double WRIST_POS_CORRECT_2ND =0.04;
    public static double WRIST_POS_CORRECT_3ND =0;
    public static double WRIST_POS_2NDPIXEL_UP=-0.05;
    public static double WRIST_POS_3NDPIXEL_UP=-0.05;
    public static double WRIST_POS_4NDPIXEL_UP=-0.04;
    public static double WRIST_POS_5NDPIXEL_UP=-0.03;// TODO LATER TRY 0.04
    public static double WRIST_DEPOSIT_NO_ARM =-0.25;

    public static double OPEN_CLAW_POS=0.6;
    public static double CLOSE_CLAW_POS=0.4;


    public static double ARM_DEAD_ZONE=0;
    public static double ARM_DOWN=0;
    public static double ARM_DEPOSIT_FRONT=1200;
    public static double ARM_DEPOSIT_BACK=3800;
    public static double ARM_DEPOSIT_BACK_AUTO = 3900;
    public static double PVARIATION=0.02;
    //revert to previous way to work TODO TODO TODO this with urgency
    public static int LVARIATION = 400;
    public static double finalArmGoal=0;

    public static double currentArmGoal=0;
    public static double armPower=0;

    public static CLAW_MODE ClawMode;
    static boolean effectiveLeft;
    static boolean effectiveRight;
    static boolean dettectedLeft;
    static boolean dettectedRight;
    static PID armPID = new PID(0.0007,0,0,0);
    static simpleSwitch LClawSwitch = new simpleSwitch();
    static simpleSwitch RClawSwitch = new simpleSwitch();

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
                finalArmGoal = ARM_DEPOSIT_BACK+ARM_DEAD_ZONE+78+50;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE+ARTICULATION_POS_DEPOSIT_BACK);
                RobotHardware.setWristPos(WRIST_MIDDLE+WRIST_POS_DEPOSIT_BACK);
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
            case CORRECT1UP:
                finalArmGoal=ARM_DOWN;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE+ARTICULATION_POS_CORRECT_1ND);
                RobotHardware.setWristPos(WRIST_MIDDLE+WRIST_POS_CORRECT_1ND);
                break;
            case CORRECT2UP:
                finalArmGoal=ARM_DOWN;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE+ARTICULATION_POS_CORRECT_2ND);
                RobotHardware.setWristPos(WRIST_MIDDLE+WRIST_POS_CORRECT_2ND);
                break;
            case CORRECT3UP:
                finalArmGoal=ARM_DOWN;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE+ARTICULATION_POS_CORRECT_3ND);
                RobotHardware.setWristPos(WRIST_MIDDLE+WRIST_POS_CORRECT_3ND);
                break;
            case DEPOSIT_NO_ARM:
                finalArmGoal=ARM_DOWN;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE+ARTICULATION_DEPOSIT_NO_ARM);
                RobotHardware.setWristPos(WRIST_MIDDLE+WRIST_DEPOSIT_NO_ARM);
                break;
            case DEBUG:
                finalArmGoal = ARM_DEPOSIT_FRONT+ARM_DEAD_ZONE;
                RobotHardware.setArtPosition(ARTICULATION_MIDDLE);
                RobotHardware.setWristPos(WRIST_MIDDLE);

        }

    }
    public static void ControlClawTeleop(boolean leftState,boolean rightState){

        //LClawSwitch.click(leftState);

        if (currentArmState == ARM_STATE.PIXEL1UP){
            dettectedLeft= ClawColorSensor.isPixelInClaw(RobotHardware.LeftColorSensor);
            dettectedRight=ClawColorSensor.isPixelInClaw(RobotHardware.RightColorSensor);

        } else {
            dettectedLeft=true;
            dettectedRight=true;
        }

        if(RobotHardware.mainArm.getCurrentPosition()<1000){
            if(currentArmState==ARM_STATE.PIXEL1UP){
                if (!effectiveLeft&&!leftState) {
                    effectiveLeft = LClawSwitch.click(dettectedLeft);
                }
                if (!effectiveRight&&!rightState){
                    effectiveRight = RClawSwitch.click(dettectedRight);
                }
            }
            effectiveRight = RClawSwitch.click(rightState);
            effectiveLeft= LClawSwitch.click(leftState);
        } else {
            effectiveRight = RClawSwitch.click(leftState);
            effectiveLeft= LClawSwitch.click(rightState);
        }


        //effective left means the left the driver sees, because when the claw is in the up position
        // the left and right are flipped, which generates confusion for drivers
        //so if the arm is in the back position the left and right claws are flipped
        if (effectiveLeft){
            RobotHardware.LClaw.setPosition(CLOSE_CLAW_POS);
            RobotHardware.leftGreenLed.setState(true);
            RobotHardware.leftRedLed.setState(false);
        } else {
            RobotHardware.LClaw.setPosition(OPEN_CLAW_POS);
            RobotHardware.leftGreenLed.setState(false);
            RobotHardware.leftRedLed.setState(true);
        }
        if (effectiveRight){
            RobotHardware.RClaw.setPosition(1-CLOSE_CLAW_POS);
            RobotHardware.rightGreenLed.setState(true);
            RobotHardware.rightRedLed.setState(false);
        } else {
            RobotHardware.RClaw.setPosition(1-OPEN_CLAW_POS);
            RobotHardware.rightGreenLed.setState(false);
            RobotHardware.rightRedLed.setState(true);
        }
    }
    public static void ControlLeftClaw(CLAW_STATE claw_state){
        switch (claw_state){
            case OPEN:
                RobotHardware.LClaw.setPosition(OPEN_CLAW_POS);
                RobotHardware.leftGreenLed.setState(false);
                RobotHardware.leftRedLed.setState(true);
                break;
            case CLOSED:
                RobotHardware.LClaw.setPosition(CLOSE_CLAW_POS);
                RobotHardware.leftGreenLed.setState(true);
                RobotHardware.leftRedLed.setState(false);
                break;
        }
    }
    public static void ControlRightClaw(CLAW_STATE claw_state){
        switch (claw_state){
            case OPEN:
                RobotHardware.RClaw.setPosition(1-OPEN_CLAW_POS);
                RobotHardware.rightGreenLed.setState(false);
                RobotHardware.rightRedLed.setState(true);
                break;
            case CLOSED:
                RobotHardware.RClaw.setPosition(1-CLOSE_CLAW_POS);
                RobotHardware.rightGreenLed.setState(true);
                RobotHardware.leftRedLed.setState(false);
                break;
        }
    }
    public static void setClawMode(CLAW_MODE mode){
        ClawMode = mode;
    }

    public static void AutoCloseClawSensor() {
        if (ClawMode==CLAW_MODE.SENSOR) {
            dettectedLeft = ClawColorSensor.isPixelInClaw(RobotHardware.LeftColorSensor);
            dettectedRight = ClawColorSensor.isPixelInClaw(RobotHardware.RightColorSensor);
            if (dettectedLeft&&RobotHardware.LClaw.getPosition()!=CLOSE_CLAW_POS) {
                ControlLeftClaw(CLAW_STATE.CLOSED);
            }
            if (dettectedRight&&RobotHardware.RClaw.getPosition()!=1-CLOSE_CLAW_POS) {
                ControlRightClaw(CLAW_STATE.CLOSED);
            }
        }
    }

    public static void armPIDLoop(boolean ARM_STOP_REQUESTED){

        currentArmGoal+= PVARIATION*(finalArmGoal-currentArmGoal);

        if (((currentArmGoal > finalArmGoal - LVARIATION) && (currentArmGoal < finalArmGoal + LVARIATION))) {
            currentArmGoal = finalArmGoal;
        } else {
            currentArmGoal+= PVARIATION*(finalArmGoal-currentArmGoal);
        }
        armPower= armPID.CalculatePID(RobotHardware.mainArm.getCurrentPosition(), currentArmGoal);
        if (ARM_STOP_REQUESTED) {
            RobotHardware.moveArm(-0.1);
            finalArmGoal = RobotHardware.mainArm.getCurrentPosition();
            currentArmGoal = finalArmGoal;
        } else {
            RobotHardware.moveArm(armPower);
        }

         /**/


        /*if (!((currentArmGoal > finalArmGoal - PVARIATION) && (currentArmGoal < finalArmGoal + PVARIATION))) {
            if (currentArmGoal > finalArmGoal) {
                currentArmGoal -= PVARIATION;
            } else {
                currentArmGoal += PVARIATION;
            }
        } else {
            currentArmGoal = finalArmGoal;
        }
        if (ARM_STOP_REQUESTED) {
            RobotHardware.moveArm(-0.17);
            finalArmGoal = RobotHardware.mainArm.getCurrentPosition();
            currentArmGoal = finalArmGoal;
        } else{
        armPower = armPID.CalculatePID(RobotHardware.mainArm.getCurrentPosition(), currentArmGoal);
        RobotHardware.mainArm.setPower(armPower);

        }
        /**/

    }
}
