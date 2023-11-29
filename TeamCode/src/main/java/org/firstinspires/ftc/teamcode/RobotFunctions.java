package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
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
import com.qualcomm.robotcore.hardware.TouchSensor;


import java.util.List;

public abstract class RobotFunctions extends LinearOpMode{

    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private CRServo mainArm;
    private CRServo armEncoder;
    private  Servo Articulation;
    private Servo LClaw;
    private Servo RClaw;
    private Servo PlaneLauncher;
    private TouchSensor bottomLimit;
    //private Servo servoArticulacao;
    //private Servo servoGarra;
    private IMU imu;
    YawPitchRollAngles orientation;
    SampleMecanumDrive Autodrive;
    double drive=0;
    double turn=0;
    double strafe=0;
    double raw_drive=0;
    double raw_turn=0;
    double raw_strafe=0;
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
    //PID pid_braco = new PID(0.005,0.01,0.003);
    double prevtime=0;
    double Artpos =0;
    boolean autoGrab = false;
    boolean AutoGrabDone=true;
    boolean IMUWorking = true;
    boolean encodersWorking = true;
    public enum DriveBaseExecMode {
        FULL_FUNCTION_MODE,
        NO_IMU_MODE,
        NO_ENCODER_MODE,
        GRAPH_MODE,
        AUTO_ALIGN
    }
    DriveBaseExecMode currentBaseExecMode = DriveBaseExecMode.FULL_FUNCTION_MODE;
    List<LynxModule> allHubs;
    SampleMecanumDrive autodrive;

    public class Control{
        DriveBase driving;
        public void hardwareMapAll(){
            autodrive = new SampleMecanumDrive(hardwareMap);
            //inicializa hardware
            imu = hardwareMap.get(IMU.class, "imu");
            frontRight = hardwareMap.get(DcMotor.class, "FRmotor");
            backRight = hardwareMap.get(DcMotor.class, "BRmotor");
            frontLeft = hardwareMap.get(DcMotor.class, "FLmotor");
            backLeft = hardwareMap.get(DcMotor.class, "BLmotor");
            mainArm = hardwareMap.get(CRServo.class, "mainArm");
            bottomLimit= hardwareMap.get(TouchSensor.class,"bottomLimitSensor");

            Articulation = hardwareMap.get(Servo.class,"Articulation");
            LClaw = hardwareMap.get(Servo.class,"LClaw");
            RClaw = hardwareMap.get(Servo.class,"RClaw");
            PlaneLauncher = hardwareMap.get(Servo.class,"PlaneLauncher");



            //corrige orientação dos motores
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

            //falsifica um encoder pro braço

            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            //servoArticulacao.setDirection(DcMotorSimple.Direction.REVERSE);


            //define variaveis essenciais e inicializa funções

            YawPitchRollAngles orientation = null;
            AngularVelocity angularVelocity;
            PlaneLauncher.setPosition(0.5);

            allHubs = hardwareMap.getAll(LynxModule.class);
        }
        private void bulkReadInputs(){
            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
            //get gamepad axis
            if (gamepad1.left_bumper){
                drive = gamepad1.left_stick_y*-1*0.4;
                turn = gamepad1.right_stick_x*0.4;
                strafe = gamepad1.left_stick_x*0.4;
            } else {
                drive = gamepad1.left_stick_y * -1;
                turn = gamepad1.right_stick_x;
                strafe = gamepad1.left_stick_x;
            }


            if (!autoGrab&&gamepad1.right_bumper){
                AutoGrabDone=false;
            }
            autoGrab= gamepad1.right_bumper;

            orientation = imu.getRobotYawPitchRollAngles();
            DeltaAngle =360- (orientation.getYaw(AngleUnit.RADIANS)*180/3.141);
        }
        private void addTelemetry(){
            telemetry.addData("CurrentExecMode",currentBaseExecMode);
            telemetry.update();
            /*Description:
            this funtion adds all debugging data needed to the telemetry board
            */

            //imu
            /*
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
            telemetry.addData("raw TR",raw_turn);
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
            /*
            telemetry.addData("bL encoder",backLeft.getCurrentPosition());
            telemetry.addData("bL encoder",backRight.getCurrentPosition());
            telemetry.addData("bL encoder",frontLeft.getCurrentPosition());
            telemetry.addData("bL encoder",frontRight.getCurrentPosition());
            /**/
            //telemetry.update(); // adiciona tudo da telemetria
        }
        private void setMode(){

            if (!(gamepad1.b||gamepad1.right_bumper) && IMUWorking &&encodersWorking){
                currentBaseExecMode = DriveBaseExecMode.FULL_FUNCTION_MODE;
            }
            if (gamepad1.right_bumper && IMUWorking && encodersWorking){
                currentBaseExecMode = DriveBaseExecMode.AUTO_ALIGN;
            }
            if (gamepad1.b && IMUWorking && encodersWorking){
                currentBaseExecMode = DriveBaseExecMode.GRAPH_MODE;
            }
        }
        private void useMode(){
            switch (currentBaseExecMode){

                case FULL_FUNCTION_MODE:
                    driving.moveBaseIMU();
                    break;
                case NO_IMU_MODE:
                    driving.moveBaseNoIMU();
                    break;
                case AUTO_ALIGN:
                    driving.autoalign();
                    break;
                case GRAPH_MODE:
                    driving.graphMode();
                    break;
            }
        }
    }
    public class DriveBase {
        void manageIMU() {

            if (gamepad1.a) {
                imu.resetYaw();
                CurrentFront = orientation.getYaw(AngleUnit.RADIANS);
            }

            //ao girar manualmente, reseta a orientação tida como frente pelo robô (para uso no PID)
            if (Math.abs(turn) >= 0.001) {
                CurrentFront = orientation.getYaw(AngleUnit.RADIANS);
                pid_turn.cummulativeError = 0;
            }


            //Parametros para o PID são calculados (não é tudo feito dentro do PID para permitir uso na telemetria)
            CurrentAngle = orientation.getYaw(AngleUnit.RADIANS);
            if (CurrentAngle < CurrentFront) {
                CurrentAngle = 2 * Math.PI + CurrentAngle - CurrentFront;
            } else {
                CurrentAngle -= CurrentFront;
            }

            //se não houver uma curva manual, um PID é usado para manter a orientação do robô
            if (Math.abs(turn) < 0.001) {
                turn = pid_turn.CalculatePID(ConvertAngle(CurrentAngle), 0, true) * -1;
                if (Math.abs(turn) < 0.1) {
                    turn = 0;
                }
            }
        }
        void moveBaseNoIMU(){
            //define as velocidades de cada motor
            double maximo = Math.max(drive+strafe + Math.abs(turn), 1);


            fL = (drive + strafe + turn) / maximo;
            fR = (drive - strafe - turn) / maximo;
            bL = (drive - strafe + turn) / maximo;
            bR = (drive + strafe - turn) / maximo;
            //geração de uma aceleração ao controlar o maximo dos motores
            //aplica as potências aos motores

            frontRight.setPower(fR * 0.5);
            frontLeft.setPower(fL * 0.5);
            backRight.setPower(bR * 0.5);
            backLeft.setPower(bL * 0.5);
        }
        void moveBaseIMU() {
                //muda a direção de movimento com base na orientação ro robo
                double rotX = strafe * Math.cos(orientation.getYaw(AngleUnit.RADIANS)) + drive * Math.sin(orientation.getYaw(AngleUnit.RADIANS));
                double rotY = strafe * Math.sin(-orientation.getYaw(AngleUnit.RADIANS)) + drive * Math.cos(-orientation.getYaw(AngleUnit.RADIANS));

                //define as velocidades de cada motor
                double maximo = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(turn), 1);


                fL = (rotY + rotX + turn) / maximo;
                fR = (rotY - rotX - turn) / maximo;
                bL = (rotY - rotX + turn) / maximo;
                bR = (rotY + rotX - turn) / maximo;

                //geração de uma aceleração ao controlar o maximo dos motores
                //aplica as potências aos motores

                frontRight.setPower(fR * 0.5);
                frontLeft.setPower(fL * 0.5);
                backRight.setPower(bR * 0.5);
                backLeft.setPower(bL * 0.5);/**/

        }
        void graphMode(){}
        void autoalign(){
            autodrive.setPoseEstimate(new Pose2d(0,0,0));
            Artpos = 0.71;
            Articulation.setPosition(0.71);
            autodrive.followTrajectory(autodrive.trajectoryBuilder(new Pose2d(0,0,0))
                    .back(2)
                    .build());
            AutoGrabDone=true;
        }

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
    }
    public class Arm{
        private void braco() {
            //alto = 540?
            //baixo = 0
        /*if (gamepad2.y||(yOn)) {
            yOn=false;
            yOn =  gamepad2.y;
            if (yOn&&gamepad2.y){
                mainArm.setPower(0);

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
            }*//*
            //powerbraco = pid_braco.CalculatePID(backRight.getCurrentPosition(),goalbraco,false);

        }*/
            if (gamepad2.left_bumper){
                PlaneLauncher.setPosition(0);
            }
            if (Math.abs(gamepad2.right_trigger)>Math.abs(gamepad2.left_trigger)){
                mainArm.setPower(gamepad2.right_trigger*0.4);
            }
            else {
                if (!bottomLimit.isPressed()) {
                    mainArm.setPower(-gamepad2.left_trigger*0.4);
                } else{
                    mainArm.setPower(0.07);
                }
            }
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
    }
}
