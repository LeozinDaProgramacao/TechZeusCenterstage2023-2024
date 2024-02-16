package org.firstinspires.ftc.teamcode.oldcode;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "CenterstageMain")

public class CenterstageMain extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private CRServo mainArm;
    private Encoder armEncoder;
    private Servo Articulation;
    private Servo LClaw;
    private Servo RClaw;
    private Servo Wrist;
    //private Servo servoArticulacao;
    //private Servo servoGarra;
    private IMU imu;
    YawPitchRollAngles orientation;
    public static int GPVARIATION=6;
    double drive=0;
    double turn=0;
    double strafe=0;
    double DeltaAngle;
    double CurrentFront;
    double CurrentAngle;
    double fL=0;
    double fR=0;
    double bL=0;
    double bR=0;
    double DfL=0;
    double DfR=0;
    double DbL=0;
    double DbR=0;
    boolean yOn;
    double powerbraco;
    double finalgoalbraco = 0;
    double goalbraco;
    //PID pid_turn = new PID(0.5,0.005,0.0,0.4);
    PID pid_turn = new PID(0.6,0.000,0.0,0.3);
    PID pid_braco = new PID(0.003,0,0,0);
    double prevtime=0;
    double Artpos =0;

    /**
     * This function is executed when this Op Mode is selected.
     */
    SampleMecanumDrive autodrive;
    @Override
    public void runOpMode() {
        autodrive = new SampleMecanumDrive(hardwareMap);
        //inicializa hardware
        imu = hardwareMap.get(IMU.class, "imu");
        frontRight = hardwareMap.get(DcMotor.class, "FRmotor");
        backRight = hardwareMap.get(DcMotor.class, "BRmotor");
        frontLeft = hardwareMap.get(DcMotor.class, "FLmotor");
        backLeft = hardwareMap.get(DcMotor.class, "BLmotor");
        mainArm = hardwareMap.get(CRServo.class, "BRACO_TEMP_MUDAR_DPS");

        Articulation = hardwareMap.get(Servo.class,"Articulation");
        LClaw = hardwareMap.get(Servo.class,"LClaw");
        RClaw = hardwareMap.get(Servo.class,"RClaw");
        Wrist = hardwareMap.get(Servo.class,"Wrist");
        mainArm = hardwareMap.get(CRServo.class, "BRACO_TEMP_MUDAR_DPS");



        //corrige orientação dos motores
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        //falsifica um encoder pro braço
        /*
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
        /*
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //define variaveis essenciais e inicializa funções

        YawPitchRollAngles orientation = null;
        AngularVelocity angularVelocity;

        waitForStart();

        double CurrentFront = 0;

        while  (opModeIsActive()) {
            braco();
            garra();
            inputs();
            manageIMU();
            moveBase();
            adicionarTelemetria();
        }
    }
    private void inputs(){
        //get gamepad axis
        /*
        raw_drive = gamepad1.left_stick_y*-1;
        raw_turn = gamepad1.right_stick_x;
        raw_strafe = gamepad1.left_stick_x;
        drive = Accelerator(raw_drive,drive);
        turn = Accelerator(raw_turn,turn);
        strafe = Accelerator(raw_strafe,strafe);
        */
        if (gamepad1.left_bumper){
            drive = gamepad1.left_stick_y*-1*0.4;
            turn = gamepad1.right_stick_x*0.4;
            strafe = gamepad1.left_stick_x*0.4;
        } else {
            drive = gamepad1.left_stick_y * -1;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;
        }

        orientation = imu.getRobotYawPitchRollAngles();
        DeltaAngle =360- (orientation.getYaw(AngleUnit.RADIANS)*180/3.141);
    }
    private void manageIMU(){

        if(gamepad1.a){
            imu.resetYaw();
            CurrentFront = orientation.getYaw(AngleUnit.RADIANS);
        }

        //ao girar manualmente, reseta a orientação tida como frente pelo robô (para uso no PID)
        if (Math.abs(turn)>=0.001){
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
        if (Math.abs(turn)<0.001){
            turn = pid_turn.CalculatePID(ConvertAngle(CurrentAngle),0,true)*-1;
            if (Math.abs(turn)<0.1){
        turn =0;
            }
        }
    }
    private void moveBase(){
        //muda a direção de movimento com base na orientação ro robo
        double rotX = strafe*Math.cos(orientation.getYaw(AngleUnit.RADIANS))+drive*Math.sin(orientation.getYaw(AngleUnit.RADIANS));
        double rotY = strafe*Math.sin(-orientation.getYaw(AngleUnit.RADIANS))+drive*Math.cos(-orientation.getYaw(AngleUnit.RADIANS));

        //define as velocidades de cada motor
        double maximo = Math.max(Math.abs(rotX)+Math.abs(rotY)+Math.abs(turn),1);


        /*DfL= (rotY+rotX+turn)/maximo;
        DfR=(rotY-rotX-turn)/maximo;
        DbL=(rotY-rotX+turn)/maximo;
        DbR=(rotY+rotX-turn)/maximo;
        fL = Accelerator(DfL,fL);
        fR = Accelerator(DfR,fR);
        bL = Accelerator(DbL,bL);
        bR = Accelerator(DbR,bR);
        /**/


        fL= (rotY+rotX+turn)/maximo;
        fR=(rotY-rotX-turn)/maximo;
        bL=(rotY-rotX+turn)/maximo;
        bR=(rotY+rotX-turn)/maximo;
        /**/
        //geração de uma aceleração ao controlar o maximo dos motores
        //aplica as potências aos motores

        frontRight.setPower(fR*0.5);
        frontLeft.setPower(fL*0.5);
        backRight.setPower(bR*0.5);
        backLeft.setPower(bL*0.5);/**/

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

      telemetry.addData("raw FB",-1*gamepad1.left_stick_y);
      telemetry.addData("raw LR",gamepad1.left_stick_x);
      telemetry.addData("raw TR",gamepad1.right_stick_x);
      telemetry.addData("1",fL);
        telemetry.addData("2",fR);
        telemetry.addData("3",bL);
        telemetry.addData("4",bR);
      //telemetry.addData("multiplier",maximo);//garante que a velocidade maxima seja 1/**/



        //velocidades aplicadas nas rodas
        /*
        telemetry.addData("delay=",getRuntime()-prevtime);
        prevtime=getRuntime();
        telemetry.addData("fL",fL);
        telemetry.addData("fR",fR);
        telemetry.addData("bL",bL);
        telemetry.addData("bR",bR);
        telemetry.addData("armpos",backRight.getCurrentPosition());
        telemetry.addData("goalbraco",goalbraco);
        telemetry.addData("powerbraco",powerbraco);
        //telemetry.addData("LARM power",LArm.getPower());
        //telemetry.addData("RARM power",RArm.getPower());
        /**/


        //Encoders

        telemetry.addData("bL encoder",backLeft.getCurrentPosition());
        telemetry.addData("bL encoder",backRight.getCurrentPosition());
        telemetry.addData("bL encoder",frontLeft.getCurrentPosition());
        telemetry.addData("bL encoder",frontRight.getCurrentPosition());
        /**/
        telemetry.update(); // adiciona tudo da telemetria
    }
    private void braco() {
        //atras = 1700
        //frente = 700
        //baixado = 0
        //kp = 0.01
        //PVar = 6


        if (gamepad2.dpad_down){
            finalgoalbraco = 0;
        }
        if (gamepad2.dpad_up){
            finalgoalbraco = 1700;
        }
        //Articulation.setPosition(gamepad2.right_stick_x);
        //Arm2.setPosition(gamepad2.left_stick_x);
        if (gamepad2.x){
            Articulation.setPosition(0.15);
            Wrist.setPosition(0.35);
            //0.72 extend
            //0.35 gradado
        }
        if (gamepad2.dpad_left){
            Articulation.setPosition(0.72);
            Wrist.setPosition(0.05);

        }
        if (gamepad2.dpad_right){
            Articulation.setPosition(0.6-gamepad2.right_trigger*0.4);
            Wrist.setPosition(0.37+gamepad2.right_trigger*0.35);
            //0.35
        }

        //goalbraco+= GPVARIATION*(finalgoalbraco-goalbraco);
        //goalbraco = finalgoalbraco;
        if (!((goalbraco>finalgoalbraco-GPVARIATION)&&(goalbraco<finalgoalbraco+GPVARIATION))){
            if (goalbraco>finalgoalbraco){
                goalbraco-=GPVARIATION;
            }else{
                goalbraco+=GPVARIATION;
            }
        } else {
            goalbraco = finalgoalbraco;
        }
        powerbraco = pid_braco.CalculatePID(frontLeft.getCurrentPosition(),goalbraco,false);

        mainArm.setPower(powerbraco);


        }

    private void garra(){
            if (gamepad2.x) {
                openClaw();
            } else if (gamepad2.y) {
                closeClaw();
            }
            if (gamepad2.dpad_down) {
                Artpos = 0.71;
            } else if (gamepad2.dpad_up) {
                Artpos = 0.45;
            } else if (gamepad2.dpad_left) {
                Artpos = 0.64;
            } else if (gamepad2.dpad_right){
                Artpos =0;
                closeClaw();
            }
            Articulation.setPosition(Artpos + gamepad2.right_stick_y * 0.1);
    }
    private void closeClaw(){
        RClaw.setPosition(0.3);
        LClaw.setPosition(0.7);
    }
    private void openClaw(){
        RClaw.setPosition(0.5);
        LClaw.setPosition(0.5);
    }
    private double Accelerator(double desired,double current){
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

    public double ConvertAngle(double angle){
        double return_angle = angle;
        if(angle>Math.PI){
            return_angle -= 2*Math.PI;
        }
        return return_angle;
    }
}
