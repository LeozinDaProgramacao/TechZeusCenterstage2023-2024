package org.firstinspires.ftc.teamcode.CentersTop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ArmMovement extends LinearOpMode {
    public void braco() {
        //atras = 1700
        //frente = 700
        //baixado = 0
        //kp = 0.01
        //PVar = 6


        if (gamepad2.dpad_down){
            DV.finalgoalbraco = 0;
        }
        if (gamepad2.dpad_up){
            DV.finalgoalbraco = 1700;
        }
        //Articulation.setPosition(gamepad2.right_stick_x);
        //DV.Articulation.setPosition(gamepad2.left_stick_x);
        if (gamepad2.x){
            DV.Articulation.setPosition(0.15);
            DV.Wrist.setPosition(0.35);
            //0.72 extend
            //0.35 gradado
        }
        if (gamepad2.dpad_left){
            DV.Articulation.setPosition(0.72);
            DV.Wrist.setPosition(0.05);

        }
        if (gamepad2.dpad_right){
            DV.Articulation.setPosition(0.6-gamepad2.right_trigger*0.4);
            DV.Wrist.setPosition(0.37+gamepad2.right_trigger*0.35);
            //0.35
        }

        //goalbraco+= GPVARIATION*(DV.finalgoalbraco-goalbraco);
        //goalbraco = DV.finalgoalbraco;
        if (!((DV.goalbraco>DV.finalgoalbraco-DV.GPVARIATION)&&(DV.goalbraco<DV.finalgoalbraco+DV.GPVARIATION))){
            if (DV.goalbraco>DV.finalgoalbraco){
                DV.goalbraco-=DV.GPVARIATION;
            }else{
                DV.goalbraco+=DV.GPVARIATION;
            }
        } else {
            DV.goalbraco = DV.finalgoalbraco;
        }
        DV.powerbraco = DV.pid_braco.CalculatePID(DV.frontRight.getCurrentPosition(),DV.goalbraco,false);

        DV.mainArm.setPower(DV.powerbraco);

        //if (!DV.bottomLimit.isPressed()){
        //    DV.frontRight.setPower(DV.powerbraco);}
        //else if (DV.powerbraco>=0) {
        //    frontRight.setPower(powerbraco);
            //WROOOONNNGGGG
        //}

    }
    public void ClawControl(){



        if (DV.leftClawSwitch.click(gamepad2.left_bumper)) {
            openLeftClaw();
        } else {
            closeLeftClaw();
        }
        if (DV.rightClawSwitch.click(gamepad2.right_bumper)) {
            openRightClaw();
        } else {
            closeRightClaw();
        }

    }
    public  void closeLeftClaw(){
        DV.LClaw.setPosition(0.3);
    }
    public  void openLeftClaw(){
        DV.LClaw.setPosition(0.7);
    }
    public  void closeRightClaw(){DV.RClaw.setPosition(0.7);
    }
    public  void openRightClaw(){
        DV.RClaw.setPosition(0.3);
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}