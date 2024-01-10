package org.firstinspires.ftc.teamcode;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.List;
@TeleOp(name = "Robot Functions")

public class RobotFunctions extends LinearOpMode{


    private static DcMotor frontRight;
    private static DcMotor backRight;
    private static DcMotor frontLeft;
    private static DcMotor backLeft;
    private Servo Arm2;
    private  Servo Articulation;
    private Servo LClaw;
    private Servo RClaw;
    private static Servo PlaneLauncher;
    private TouchSensor bottomLimit;
    //private Servo servoArticulacao;
    //private Servo servoGarra;
    private IMU imu;
    DigitalChannel greenLED;
    DigitalChannel redLED;
    DistanceSensor distanceSensor;
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
    public static int GPVARIATION=6;
    public static int MAXHEIGHT=1700;

    PID pid_braco = new PID(0.003,0,0,0);
    double prevtime=0;
    double Artpos =0;
    boolean autoGrab = false;
    boolean AutoGrabDone=true;
    boolean IMUWorking = true;
    boolean encodersWorking = true;
    simpleSwitch leftClawSwitch = new simpleSwitch();
    simpleSwitch rightClawSwitch = new simpleSwitch();

    Control ctr = new Control();

    @Override
    public void runOpMode() throws InterruptedException {
        ctr.hardwareMapAll();
        waitForStart();
        ctr.startup();
        ElapsedTime timeElapsed = new ElapsedTime();
        while  (opModeIsActive()) {
            telemetry.addData("nano",timeElapsed.nanoseconds());
            ctr.cycleLoop();
            timeElapsed.reset();

        }
    }

    public enum DriveBaseExecMode {
        FULL_FUNCTION_MODE,
        BACKDROP_FULL_FUNCTION_MODE,
        NO_IMU_MODE,
        NO_ENCODER_MODE,
        GRAPH_MODE,
        AUTO_ALIGN
    }
    DriveBaseExecMode currentBaseExecMode = DriveBaseExecMode.FULL_FUNCTION_MODE;
    static List<LynxModule> allHubs;
    public SampleMecanumDrive autodrive;

    public class Control{
        DriveBase driving = new DriveBase();

        Arm arm = new Arm();

        public void StatusLights(DriveBaseExecMode status){

            //TODO make this better with switch cases
            //blank if all is well
            /*if (status = DriveBaseExecMode.FULL_FUNCTION_MODE){
                redLED.setState(false);
                greenLED.setState(false);
            }
            if (positionZeroed){
                redLED.setState(false);
                greenLED.setState(true);
            }
            if ((status != DriveBaseExecMode.FULL_FUNCTION_MODE)&&(status!=DriveBaseExecMode))
            */



            //green if dettect april tag
            //red if limited function mode
            //amber not used because too close to red

        }
        public void startup(){
            redLED.setMode(DigitalChannel.Mode.OUTPUT);
            greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        }
        public void cycleLoop(){
            if (gamepad1.a){
                greenLED.setState(false);
                redLED.setState(true);
            }
            if (gamepad1.y){
                greenLED.setState(true);
                redLED.setState(false);
            }
            if (gamepad1.b){
                greenLED.setState(false);
                redLED.setState(false);
            }
            if (gamepad1.x){
                greenLED.setState(true);
                redLED.setState(true);
            }
            bulkReadInputs();
            driving.moveBaseNoIMU();
            arm.armheight();
            arm.braco();
            arm.ClawControl();
            addTelemetry();
        }

        public void hardwareMapAll(){

            autodrive = new SampleMecanumDrive(hardwareMap);
            //inicializa hardware
            imu = hardwareMap.get(IMU.class, "imu");
            frontRight = hardwareMap.get(DcMotor.class, "FRmotor");
            backRight = hardwareMap.get(DcMotor.class, "BRmotor");
            frontLeft = hardwareMap.get(DcMotor.class, "FLmotor");
            backLeft = hardwareMap.get(DcMotor.class, "BLmotor");
            bottomLimit= hardwareMap.get(TouchSensor.class,"bottomLimitSensor");

            Articulation = hardwareMap.get(Servo.class,"Articulation");
            Arm2 = hardwareMap.get(Servo.class,"Arm2");
            LClaw = hardwareMap.get(Servo.class,"LClaw");
            RClaw = hardwareMap.get(Servo.class,"RClaw");
            PlaneLauncher = hardwareMap.get(Servo.class,"PlaneLauncher");

            redLED = hardwareMap.get(DigitalChannel.class, "redL");
            greenLED = hardwareMap.get(DigitalChannel.class, "greenL");

            distanceSensor = hardwareMap.get(DistanceSensor.class,"distSensor");



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
            //TODO make this using encoder when can
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            /*Description:
            this funtion adds all debugging data needed to the telemetry board
            */
            telemetry.addData("CurrentExecMode",currentBaseExecMode);
            telemetry.addData("pb",powerbraco);
            telemetry.addData("pos",frontLeft.getCurrentPosition());
            telemetry.addData("distance",distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("error",pid_braco.error);
            telemetry.addData("gm2 lt",gamepad2.left_stick_x);
            telemetry.addData("gm2 rt",gamepad2.right_stick_x);
            telemetry.update();
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
            telemetry.addData("bR encoder",backRight.getCurrentPosition());
            telemetry.addData("fL encoder",frontLeft.getCurrentPosition());
            telemetry.addData("fR encoder",frontRight.getCurrentPosition());
            /**/
            //telemetry.update(); // adiciona tudo da telemetria
        }
        private void setMode(){

            if (!(gamepad1.b||gamepad1.right_bumper) && IMUWorking &&encodersWorking){
                currentBaseExecMode = DriveBaseExecMode.FULL_FUNCTION_MODE;
            }
            if ((goalbraco>1000)&&(distanceSensor.getDistance(DistanceUnit.CM)<30)){
                currentBaseExecMode = DriveBaseExecMode.BACKDROP_FULL_FUNCTION_MODE;
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
                case BACKDROP_FULL_FUNCTION_MODE:
                    driving.moveBaseBackdropFull();
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
        public class DriveBase {
            void manageIMU() {

                if (gamepad1.a) {
                    imu.resetYaw();
                    CurrentFront = orientation.getYaw(AngleUnit.RADIANS);
                }

                //ao girar manualmente, reseta a orientação tida como frente pelo robô (para uso no PID)
                if (Math.abs(turn) >= 0.01) {
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
            public void moveBaseBackdropFull() {
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

                frontRight.setPower(fR * 0.3);
                frontLeft.setPower(fL * 0.3);
                backRight.setPower(bR * 0.3);
                backLeft.setPower(bL * 0.3);/**/
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
            private void armheight(){
                if (gamepad2.dpad_down){
                    finalgoalbraco = 0;
                }
                if (gamepad2.dpad_up){
                    finalgoalbraco = 1700;
                }
                //Articulation.setPosition(gamepad2.right_stick_x);
                //Arm2.setPosition(gamepad2.left_stick_x);
                if (gamepad2.x){
                    Arm2.setPosition(0.15);
                    Articulation.setPosition(0.35);
                    //0.72 extend
                    //0.35 gradado
                }
                if (gamepad2.dpad_left){
                    Arm2.setPosition(0.72);
                    Articulation.setPosition(0.05);

                }
                if (gamepad2.dpad_right){
                    Arm2.setPosition(0.6-gamepad2.right_trigger*0.4);
                    Articulation.setPosition(0.37+gamepad2.right_trigger*0.35);
                    //0.35
                }
            }
            private void braco() {
                //atras = 1700
                //frente = 700
                //baixado = 0
                //kp = 0.01
                //PVar = 6

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


                if (!bottomLimit.isPressed()){
                    frontLeft.setPower(powerbraco);}
                else if (powerbraco>=0) {
                    frontLeft.setPower(powerbraco);
                }

            }
            private void ClawControl(){



                if (leftClawSwitch.click(gamepad2.left_bumper)) {
                    openLeftClaw();
                } else {
                    closeLeftClaw();
                }
                if (rightClawSwitch.click(gamepad2.right_bumper)) {
                    openRightClaw();
                } else {
                    closeRightClaw();
                }

            }
            private void closeLeftClaw(){
                LClaw.setPosition(0.3);
            }
            private void openLeftClaw(){
                LClaw.setPosition(0.7);
            }
            private void closeRightClaw(){
                RClaw.setPosition(0.7);
            }
            private void openRightClaw(){
                RClaw.setPosition(0.3);
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
    public class simpleSwitch{
        private boolean previouslyPressed=false;
        private boolean currentState = false;
        boolean click(boolean pressed) {
            if (!previouslyPressed && pressed) {
                currentState = !currentState;
            }
            previouslyPressed = pressed;
            return currentState;
        }
    }

}
