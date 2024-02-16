
package org.firstinspires.ftc.teamcode.oldcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Config
@Disabled
@TeleOp(name = "leozinconfig")

public class leozinconfig extends LinearOpMode {
    FtcDashboard maromba;
    public static double KP=0.01;
    public static double PVARIATION=0.037;
    public static double KI=0.0006;
    public static double KD=0;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private CRServo LArm;
    private CRServo RArm;
    private Servo servoArticulacao;
    private Servo servoGarra;
    private IMU imu;
    YawPitchRollAngles orientation;
    double drive;
    double turn;
    double strafe;
    double DeltaAngle;
    double CurrentFront;
    double CurrentAngle;
    double fL;
    double fR;
    double bL;
    double bR;
    boolean yOn;
    double powerbraco;
    double finalgoalbraco = 0;
    double goalbraco;
    final double maximoo = 0.5;
    //PID pid_turn = new PID(0.5,0.005,0.0,0.4);
    PID pid_turn = new PID(0.5,0.000,0.0,0.3);
    //PID pid_braco = new PID(0.005,0.01,0.003);
    PID pid_braco = new PID(0.008,0.0006,0,0.4);
    double prevtime=0;

    /**
     * This function is executed when this Op Mode is selected.
     */
    @Override
    public void runOpMode() {
        maromba =FtcDashboard.getInstance();
        //inicializa hardware
        imu = hardwareMap.get(IMU.class, "imu");
        frontRight = hardwareMap.get(DcMotor.class, "FRmotor");
        backRight = hardwareMap.get(DcMotor.class, "BRmotor");
        frontLeft = hardwareMap.get(DcMotor.class, "FLmotor");
        backLeft = hardwareMap.get(DcMotor.class, "BLmotor");
        LArm = hardwareMap.get(CRServo.class, "LArm");
        RArm = hardwareMap.get(CRServo.class, "RArm");
        servoArticulacao = hardwareMap.get(Servo.class, "servoArticulacao");
        servoGarra = hardwareMap.get(Servo.class, "servoGarra");


        //corrige orientação dos motores
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        RArm.setDirection(DcMotorSimple.Direction.REVERSE);
        LArm.setDirection(DcMotorSimple.Direction.FORWARD);

        //falsifica um encoder pro braço
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double CurrentFront = 0;

        TelemetryPacket pack1 = new TelemetryPacket();
        while  (opModeIsActive()) {
            pack1.put("armpos",backRight.getCurrentPosition());
            pack1.put("goal",pid_braco.goal);
            pack1.put("final goal",finalgoalbraco);
            maromba.sendTelemetryPacket(pack1);

            pid_braco.kP = KP;
            pid_braco.kI = KI;
            pid_braco.kD = KD;
            braco();
            garra();
            //get gamepad axis
            inputs();
            manageIMU();
            moveBase();
            adicionarTelemetria();
        }
    }
    private void inputs(){
        //get gamepad axis
        drive = gamepad1.left_stick_y*-1;
        turn = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        orientation = imu.getRobotYawPitchRollAngles();
        DeltaAngle =360- (orientation.getYaw(AngleUnit.RADIANS)*180/3.141);
    }
    private void manageIMU(){

        if(gamepad1.a){
            imu.resetYaw();
            CurrentFront = orientation.getYaw(AngleUnit.RADIANS);
        }

        //ao girar manualmente, reseta a orientação tida como frente pelo robô (para uso no PID)
        if (Math.abs(gamepad1.right_stick_x)>=0.09){
            CurrentFront = orientation.getYaw(AngleUnit.RADIANS);
            pid_turn.cummulativeError=0;

        }



        //Parametros para o PID são calculados (não é tudo feito dentro do PID para permitir uso na telemetria)
        CurrentAngle = orientation.getYaw(AngleUnit.RADIANS);
        if (CurrentAngle <CurrentFront){
            CurrentAngle = 2*Math.PI+CurrentAngle-CurrentFront;
        } else{
            CurrentAngle-=CurrentFront;
        }

        //se não houver uma curva manual, um PID é usado para manter a orientação do robô
        if (Math.abs(turn)<0.09){
            turn = pid_turn.CalculatePID(ConvertAngle(CurrentAngle),0,true)*-1;
        }
    }
    private void moveBase(){
        //muda a direção de movimento com base na orientação ro robo
        double rotX = strafe*Math.cos(orientation.getYaw(AngleUnit.RADIANS))+drive*Math.sin(orientation.getYaw(AngleUnit.RADIANS));
        double rotY = strafe*Math.sin(-orientation.getYaw(AngleUnit.RADIANS))+drive*Math.cos(-orientation.getYaw(AngleUnit.RADIANS));

        //define as velocidades de cada motor
        double maximo = Math.max(Math.abs(rotX)+Math.abs(rotY)+Math.abs(turn),1);
        fL= (rotY+rotX+turn)/maximo;
        fR=(rotY-rotX-turn)/maximo;
        bL=(rotY-rotX+turn)/maximo;
        bR=(rotY+rotX-turn)/maximo;

        //aplica as potências aos motores
        frontRight.setPower(fR*maximoo*0.7);
        frontLeft.setPower(fL*maximoo*0.7);
        backRight.setPower(bR*maximoo*1);
        backLeft.setPower(bL*maximoo*1);/**/
    }
    private void adicionarTelemetria(){
      /*Description:
      this funtion adds all debugging data needed to the telemetry board
      */

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
        telemetry.addData("fL",fL);
        telemetry.addData("fR",fR);
        telemetry.addData("bL",bL);
        telemetry.addData("bR",bR);
        telemetry.addData("armpos",backRight.getCurrentPosition());
        telemetry.addData("goalbraco",goalbraco);
        telemetry.addData("erro",pid_braco.error);
        telemetry.addData("powerbraco",powerbraco);
        telemetry.addData("LARM power",LArm.getPower());
        telemetry.addData("RARM power",RArm.getPower());
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
                LArm.setPower(-0.2);
                RArm.setPower(+0.2);
            } else {
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        } else {
            finalgoalbraco = gamepad2.right_trigger*540;
            goalbraco+= PVARIATION*(finalgoalbraco-goalbraco);
            //goalbraco = finalgoalbraco;
            /*if (!((goalbraco>finalgoalbraco-1)&&(goalbraco<finalgoalbraco+1))){
                if (goalbraco>finalgoalbraco){
                    goalbraco-=1;
                }else{
                    goalbraco+=1;
                }
            }*/
            powerbraco = pid_braco.CalculatePID(backRight.getCurrentPosition(),goalbraco,false);
            LArm.setPower(powerbraco);
            RArm.setPower(-powerbraco);
        }
    }
    private void garra(){
        if (gamepad2.a) {
            servoGarra.setPosition(0);
        } else if (gamepad2.b) {
            servoGarra.setPosition(1);
        }
        if (gamepad2.dpad_down) {
            servoArticulacao.setPosition(0.5);
        } else if (gamepad2.dpad_up) {
            servoArticulacao.setPosition(1.2);
        }/**/else if (gamepad2.dpad_left) {
            servoArticulacao.setPosition(0.7);
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


