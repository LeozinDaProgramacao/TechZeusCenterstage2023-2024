package org.firstinspires.ftc.teamcode;





import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Encoder;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Config
@TeleOp(name = "PIDbraconovo")

public class PIDbraconovo extends LinearOpMode {
    FtcDashboard dashboard;
    public static double KP=0;
    public static double PVARIATION=0.037;
    public static double KI=0;
    public static double KD=0;
    public static double MAXI =0;
    public static int MAXHEIGHT=0;
    boolean yOn = false;

    private DcMotor frontLeft;

    private CRServo mainArm;
    private CRServo armEncoder;
    private TouchSensor bottomLimit;

    //private Servo servoArticulacao;
    //private Servo servoGarra;
    double powerbraco;
    double finalgoalbraco = 0;
    double goalbraco;
    final double maximoo = 0.5;
    //PID pid_turn = new PID(0.5,0.005,0.0,0.4);
    //PID pid_turn = new PID(0.0,0.000,0.0,0.3);
    //PID pid_braco = new PID(0.005,0.01,0.003);
    PID pid_braco = new PID(0.0,0.0,0,0.0);
    double prevtime=0;

    /**
     * This function is executed when this Op Mode is selected.
     */
    @Override
    public void runOpMode() {
        dashboard =FtcDashboard.getInstance();
        //inicializa hardware
        frontLeft = hardwareMap.get(DcMotor.class,"FLmotor");
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mainArm = hardwareMap.get(CRServo.class, "mainArm");
        bottomLimit= hardwareMap.get(TouchSensor.class,"bottomLimitSensor");
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

        TelemetryPacket pack1 = new TelemetryPacket();
        while  (opModeIsActive()) {
            //pack1.put("armpos",armEncoder.getController().getServoPosition(5));
            pack1.put("goal",pid_braco.goal);
            pack1.put("final goal",finalgoalbraco);
            pack1.put("posição braco",frontLeft.getCurrentPosition());

            dashboard.sendTelemetryPacket(pack1);

            pid_braco.kP = KP;
            pid_braco.kI = KI;
            pid_braco.kD = KD;
            pid_braco.MaxI = MAXI;

            braco();
            telemetry.addData("desce",gamepad2.left_trigger);
            telemetry.addData("sobe",gamepad2.right_trigger);
            telemetry.addData("posicao",frontLeft.getCurrentPosition());
            //telemetry.addData("cu",armEncoder.getController().getServoPosition(5));
            telemetry.update();
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
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        } else {
            finalgoalbraco = gamepad2.right_trigger*MAXHEIGHT;
            //goalbraco+= PVARIATION*(finalgoalbraco-goalbraco);
            //goalbraco = finalgoalbraco;
            if (!((goalbraco>finalgoalbraco-1)&&(goalbraco<finalgoalbraco+1))){
                if (goalbraco>finalgoalbraco){
                    goalbraco-=PVARIATION;
                }else{
                    goalbraco+=PVARIATION;
                }
            }
            powerbraco = pid_braco.CalculatePID(frontLeft.getCurrentPosition(),goalbraco,false);

        }
        if (!bottomLimit.isPressed()){
        mainArm.setPower(powerbraco);}
        else if (powerbraco>=0) {
            mainArm.setPower(powerbraco);
        }


    }
    private void garra(){

    }
    public double ConvertAngle(double angle){
        double return_angle = angle;
        if(angle>Math.PI){
            return_angle -= 2*Math.PI;
        }
        return return_angle;
    }
}




