package org.firstinspires.ftc.teamcode.CentersTop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveBase extends LinearOpMode {
    void manageIMU() {
        DV.orientation = DV.imu.getRobotYawPitchRollAngles();

        if (gamepad1.a) {
            DV.imu.resetYaw();
            DV.CurrentFront = DV.orientation.getYaw(AngleUnit.RADIANS);
        }

        //ao girar manualmente, reseta a orientação tida como frente pelo robô (para uso no PID)
        if (Math.abs(DV.turn) >= 0.01) {
            DV.CurrentFront = DV.orientation.getYaw(AngleUnit.RADIANS);
            DV.pid_turn.cummulativeError = 0;
        }


        //Parametros para o PID são calculados (não é tudo feito dentro do PID para permitir uso na telemetria)
        DV.CurrentAngle = DV.orientation.getYaw(AngleUnit.RADIANS);
        if (DV.CurrentAngle < DV.CurrentFront) {
            DV.CurrentAngle = 2 * Math.PI + DV.CurrentAngle - DV.CurrentFront;
        } else {
            DV.CurrentAngle -= DV.CurrentFront;
        }

        //se não houver uma curva manual, um PID é usado para manter a orientação do robô
        if (Math.abs(DV.turn) < 0.001) {
            DV.turn = DV.pid_turn.CalculatePID(ConvertAngle(DV.CurrentAngle), 0, true) * -1;
            if (Math.abs(DV.turn) < 0.1) {
                DV.turn = 0;
            }
        }
    }
    void moveBaseNoIMU(){
        //define as velocidades de cada motor
        double maximo = Math.max(DV.drive+DV.strafe + Math.abs(DV.turn), 1);


        DV.fL = (DV.drive + DV.strafe + DV.turn) / maximo;
        DV.fR = (DV.drive - DV.strafe - DV.turn) / maximo;
        DV.bL = (DV.drive - DV.strafe + DV.turn) / maximo;
        DV.bR = (DV.drive + DV.strafe - DV.turn) / maximo;
        //geração de uma aceleração ao controlar o maximo dos motores
        //aplica as potências aos motores

        DV.frontRight.setPower(DV.fR * 0.5);
        DV.frontLeft.setPower(DV.fL * 0.5);
        DV.backRight.setPower(DV.bR * 0.5);
        DV.backLeft.setPower(DV.bL * 0.5);
    }
    void moveBaseIMU() {
        //muda a direção de movimento com base na orientação ro robo
        DV.rotX = DV.strafe * Math.cos(DV.orientation.getYaw(AngleUnit.RADIANS)) + DV.drive * Math.sin(DV.orientation.getYaw(AngleUnit.RADIANS));
        DV.rotY = DV.strafe * Math.sin(-DV.orientation.getYaw(AngleUnit.RADIANS)) + DV.drive * Math.cos(-DV.orientation.getYaw(AngleUnit.RADIANS));

        //define as velocidades de cada motor
        DV.maximo = Math.max(Math.abs(DV.rotX) + Math.abs(DV.rotY) + Math.abs(DV.turn), 1);


        DV.fL = (DV.rotY + DV.rotX + DV.turn) / DV.maximo;
        DV.fR = (DV.rotY - DV.rotX - DV.turn) / DV.maximo;
        DV.bL = (DV.rotY - DV.rotX + DV.turn) / DV.maximo;
        DV.bR = (DV.rotY + DV.rotX - DV.turn) / DV.maximo;

        //geração de uma aceleração ao controlar o maximo dos motores
        //aplica as potências aos motores

        DV.frontRight.setPower(DV.fR * 0.5);
        DV.frontLeft.setPower(DV.fL * 0.5);
        DV.backRight.setPower(DV.bR * 0.5);
        DV.backLeft.setPower(DV.bL * 0.5);/**/

    }
    public void moveBaseBackdropFull() {
        //muda a direção de movimento com base na orientação ro robo
        DV.rotX = DV.strafe * Math.cos(DV.orientation.getYaw(AngleUnit.RADIANS)) + DV.drive * Math.sin(DV.orientation.getYaw(AngleUnit.RADIANS));
        DV.rotY = DV.strafe * Math.sin(-DV.orientation.getYaw(AngleUnit.RADIANS)) + DV.drive * Math.cos(-DV.orientation.getYaw(AngleUnit.RADIANS));

        //define as velocidades de cada motor
        double maximo = Math.max(Math.abs(DV.rotX) + Math.abs(DV.rotY) + Math.abs(DV.turn), 1);


        DV.fL = (DV.rotY + DV.rotX + DV.turn) / maximo;
        DV.fR = (DV.rotY - DV.rotX - DV.turn) / maximo;
        DV.bL = (DV.rotY - DV.rotX + DV.turn) / maximo;
        DV.bR = (DV.rotY + DV.rotX - DV.turn) / maximo;

        //geração de uma aceleração ao controlar o maximo dos motores
        //aplica as potências aos motores

        DV.frontRight.setPower(DV.fR * 0.3);
        DV.frontLeft.setPower(DV.fL * 0.3);
        DV.backRight.setPower(DV.bR * 0.3);
        DV.backLeft.setPower(DV.bL * 0.3);/**/
    }
    void graphMode(){}



    private double Accelerator(double desired, double current) {
        /*
        double konstant=0.03;
        if (Math.abs(desired)>Math.abs(current)&&!gamepad1.left_bumper){
            if (Math.abs(current+konstant*desired)>1){
                return Integer.signum((int)(current+konstant*desired));
            } else {
                return current + konstant*desired;
            }
        } else{
            return desired;
        }*/
        return desired;
    }

    public double ConvertAngle(double angle) {
        double return_angle = angle;
        if (angle > Math.PI) {
            return_angle -= 2 * Math.PI;
        }
        return return_angle;
    }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
