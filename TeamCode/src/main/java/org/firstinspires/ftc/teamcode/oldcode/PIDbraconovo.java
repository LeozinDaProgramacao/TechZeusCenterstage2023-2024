package org.firstinspires.ftc.teamcode.oldcode;





import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "PIDbraconovo")
//@Disabled
public class PIDbraconovo extends LinearOpMode {
    FtcDashboard dashboard;
    public static double KP=0.005;
    public static double PVARIATION=0.0001;
    public static double LVARIATION = 100;
    public static double KI=0;
    public static double KD=0;
    public static double MAXI =0;
    public static int MAXHEIGHT=1569;
    boolean yOn = false;

    private DcMotor mainArm;
    private

    //private Servo servoArticulacao;
    //private Servo servoGarra;
    double powerbraco=0;
    double finalgoalbraco = 0;
    double goalbraco;
    final double maximoo = 0.5;
    //PID pid_turn = new PID(0.5,0.005,0.0,0.4);
    //PID pid_turn = new PID(0.0,0.000,0.0,0.3);
    //PID pid_braco = new PID(0.005,0.01,0.003);
    PID pid_braco = new PID(0,0.0,0,0.0);
    double prevtime=0;

    /**
     * This function is executed when this Op Mode is selected.
     */
    @Override
    public void runOpMode() {
        dashboard =FtcDashboard.getInstance();
        //inicializa hardware
        mainArm =hardwareMap.get(DcMotor.class,"MainArm");
        //servoArticulacao = hardwareMap.get(Servo.class, "servoArticulacao");
        //servoGarra = hardwareMap.get(Servo.class, "servoGarra");


        //corrige orientação dos motores


        //servoArticulacao.setDirection(DcMotorSimple.Direction.REVERSE);


        //Ativa movimentação com Encoders (a fazer)
        //
        //
        //

        //define variaveis essenciais e inicializa funções

        YawPitchRollAngles orientation = null;
        AngularVelocity angularVelocity;

        waitForStart();

        //falsifica um encoder pro braço
        double CurrentFront = 0;
        ElapsedTime timeElapsed = new ElapsedTime();
        TelemetryPacket pack1 = new TelemetryPacket();
        while  (opModeIsActive()) {
            telemetry.addData("nano",timeElapsed.nanoseconds());
            //pack1.put("armpos",armEncoder.getController().getServoPosition(5));
            pack1.put("goal",pid_braco.goal);
            pack1.put("final goal",finalgoalbraco);
            pack1.put("posição braco",mainArm.getCurrentPosition());

            dashboard.sendTelemetryPacket(pack1);

            pid_braco.kP = KP;
            pid_braco.kI = KI;
            pid_braco.kD = KD;
            pid_braco.MaxI = MAXI;

            braco();
            telemetry.addData("desce",gamepad2.left_trigger);
            telemetry.addData("sobe",gamepad2.right_trigger);
            telemetry.addData("posicao",mainArm.getCurrentPosition());
            telemetry.addData("PODEEEERRR",powerbraco);
            telemetry.addData("powah",mainArm.getPower());
            //telemetry.addData("cu",armEncoder.getController().getServoPosition(5));
            telemetry.update();
            timeElapsed.reset();
            //adicionarTelemetria();
        }
    }
    private void inputs(){

    }
    private void manageIMU(){


    }
    private void moveBase(){

    }
    private void adicionarTelemetria(){
      /*Description:
      this funtion adds all debugging data needed to the telemetry board
      */
        /*
        //imu

        telemetry.addData("Delta Angle", DeltaAngle);//quanto a base deslocou em radianos desde o ultimo reset
        telemetry.addData("turn PID detected Error", pid_turn.error*180/Math.PI);
        telemetry.addData("current angle",CurrentAngle*180/Math.PI);
        telemetry.addData("currentfront",CurrentFront);
        telemetry.addData("cumError",pid_turn.cummulativeError);
        /* */


        //valores crus (direto do gamepad)
      /*
      telemetry.addData("raw FB",-1*gamepad1.left_stick_y);
      telemetry.addData("raw LR",gamepad1.left_stick_x);
      telemetry.addData("raw TR",turn);
      //telemetry.addData("multiplier",maximo);//garante que a velocidade maxima seja 1/**/



        //velocidades aplicadas nas rodas
        telemetry.addData("delay=",getRuntime()-prevtime);
        prevtime=getRuntime();

        telemetry.addData("goalbraco",goalbraco);
        telemetry.addData("erro",pid_braco.error);
        telemetry.addData("powerbraco",powerbraco);
        //telemetry.addData("LARM power",LArm.getPower());
        //telemetry.addData("RARM power",RArm.getPower());
        telemetry.addData("p Term",pid_braco.P(pid_braco.kP));
        telemetry.addData("i Term",pid_braco.I(pid_braco.kI));
        telemetry.addData("d Term",pid_braco.D(pid_braco.kD));
        telemetry.update(); // adiciona tudo da telemetria
    }
    private void braco() {
        //alto = 540?
        //baixo = 0
        if (gamepad2.y||(yOn)) {
            yOn=false;
            yOn =  gamepad2.y;
            if (yOn&&gamepad2.y){
                powerbraco=0;


            } else {
                mainArm.setPower(0);
            }
        } else {
            finalgoalbraco = gamepad2.right_trigger * MAXHEIGHT;
            goalbraco+= PVARIATION*(finalgoalbraco-goalbraco);
            if (((goalbraco > finalgoalbraco - LVARIATION) && (goalbraco < finalgoalbraco + LVARIATION))) {
                goalbraco = finalgoalbraco;
            } else {
                    goalbraco+= PVARIATION*(finalgoalbraco-goalbraco);
            }
            /**/




            //goalbraco = finalgoalbraco;

            /*if (!((goalbraco > finalgoalbraco - PVARIATION) && (goalbraco < finalgoalbraco + PVARIATION))) {
                if (goalbraco > finalgoalbraco) {
                    goalbraco -= PVARIATION;
                } else {
                    goalbraco += PVARIATION;
                }
            } else {
                goalbraco = finalgoalbraco;
            }
            /**/
            powerbraco = pid_braco.CalculatePID(mainArm.getCurrentPosition(), goalbraco, false);
            if (gamepad2.x) {
                mainArm.setPower(0);
            } else {
                mainArm.setPower(powerbraco);
            }

        }
    }

    public double ConvertAngle(double angle){
        double return_angle = angle;
        if(angle>Math.PI){
            return_angle -= 2*Math.PI;
        }
        return return_angle;
    }
}




