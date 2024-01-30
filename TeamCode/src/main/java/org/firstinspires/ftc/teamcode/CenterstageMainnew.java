package org.firstinspires.ftc.teamcode;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Encoder;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "CenterstageMainNew")

public class CenterstageMainnew extends LinearOpMode {
    TrajectorySequence AStarPath;

    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor mainArm;
    private DcMotor CoreEsq;
    private DcMotor CoreDir;
    private Servo Articulation;

    private Servo LClaw;
    private Servo RClaw;
    private Servo Wrist;
    //private Servo servoArticulacao;
    //private Servo servoGarra;
    private IMU imu;
    YawPitchRollAngles orientation;
    public static double PVARIATION=0.01;
    public static int LVARIATION = 100;
    public static int MAXHEIGHT = 1569;
    int startingArmPos =0;
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
    boolean graphMode = false;
    boolean pathGenerated = false;
    boolean yOn;
    double powerbraco;
    double finalgoalbraco = 0;
    double goalbraco;
    PID pid_turn = new PID(0.65,0,0,0);
    //PID pid_turn = new PID(0.5,0.005,0.0,0.4);
    //PID pid_turn = new PID(0.6,0.000,0.0,0.3);
    PID pid_braco = new PID(0.005,0,0,0);

    simpleSwitch leftClawSwitch = new simpleSwitch();
    simpleSwitch rightClawSwitch = new simpleSwitch();
    double prevtime=0;
    double Artpos =0;
    public static List<Node> allNodes= new ArrayList<>();
    double goalX=0;

    /**
     * This function is executed when this Op Mode is selected.
     */

    SampleMecanumDriveCancelable autodrive;
    @Override
    public void runOpMode() {

        //inicializa hardware
        imu = hardwareMap.get(IMU.class, "imu");
        frontRight = hardwareMap.get(DcMotor.class, "FRmotor");
        backRight = hardwareMap.get(DcMotor.class, "BRmotor");
        frontLeft = hardwareMap.get(DcMotor.class, "FLmotor");
        backLeft = hardwareMap.get(DcMotor.class, "BLmotor");
        mainArm = hardwareMap.get(DcMotor.class, "MainArm");


        Articulation = hardwareMap.get(Servo.class,"Articulation");
        LClaw = hardwareMap.get(Servo.class,"LClaw");
        RClaw = hardwareMap.get(Servo.class,"RClaw");
        Wrist = hardwareMap.get(Servo.class,"Wrist");



        //corrige orientação dos motores
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //define variaveis essenciais e inicializa funções

        YawPitchRollAngles orientation = null;
        AngularVelocity angularVelocity;
        mainArm.setPower(-0.2);
        startingArmPos = mainArm.getCurrentPosition();
        waitForStart();
        mainArm.setPower(0);

        double CurrentFront = 0;
        autodrive = new SampleMecanumDriveCancelable(hardwareMap);
        autodrive.setPoseEstimate(new Pose2d(24,0,0));
        generateGraph();
        while  (opModeIsActive()) {
            braco();
            inputs();
            moveBase();
            adicionarTelemetria();
            if (gamepad2.right_trigger>0.5){
                frontLeft.setPower(0.1);
                frontRight.setPower(0.1);
                backLeft.setPower(0.1);
                backRight.setPower(0.1);
            }
            if (gamepad2.a){
                frontLeft.setPower(0.1);
            }
            if (gamepad2.b){
                frontRight.setPower(0.1);
            }
            if (gamepad2.y){
                backLeft.setPower(0.1);
            }
            if (gamepad2.start){
                backRight.setPower(0.1);
            }

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
        autodrive.update();
        if (gamepad1.dpad_right){
        graphMode=true;
        }
        if (!graphMode||gamepad1.dpad_left) {
            if (gamepad1.dpad_left){
                autodrive.breakFollowing();
            }
            graphMode=false;
            pathGenerated=false;

            manageIMU();

            //muda a direção de movimento com base na orientação ro robo
            double rotX = strafe * Math.cos(orientation.getYaw(AngleUnit.RADIANS)) + drive * Math.sin(orientation.getYaw(AngleUnit.RADIANS));
            double rotY = strafe * Math.sin(-orientation.getYaw(AngleUnit.RADIANS)) + drive * Math.cos(-orientation.getYaw(AngleUnit.RADIANS));

            //define as velocidades de cada motor
            double maximo = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(turn), 1);


        /*DfL= (rotY+rotX+turn)/maximo;
        DfR=(rotY-rotX-turn)/maximo;
        DbL=(rotY-rotX+turn)/maximo;
        DbR=(rotY+rotX-turn)/maximo;
        fL = Accelerator(DfL,fL);
        fR = Accelerator(DfR,fR);
        bL = Accelerator(DbL,bL);
        bR = Accelerator(DbR,bR);
        /**/


            fL = (rotY + rotX + turn) / maximo;
            fR = (rotY - rotX - turn) / maximo;
            bL = (rotY - rotX + turn) / maximo;
            bR = (rotY + rotX - turn) / maximo;
            /**/
            //geração de uma aceleração ao controlar o maximo dos motores
            //aplica as potências aos motores

            frontRight.setPower(fR * 0.75);
            frontLeft.setPower(fL * 0.75);
            backRight.setPower(bR * 0.75);
            backLeft.setPower(bL * 0.75);/**/
        } else{
            if (!pathGenerated){
                resetGraph();
                TrajectorySequence AStarPath= Node.printPath(new Node(autodrive.getPoseEstimate().getX(),autodrive.getPoseEstimate().getY()),allNodes,autodrive);
                goalX = Node.getTarget(new Node(autodrive.getPoseEstimate().getX(),autodrive.getPoseEstimate().getY()),allNodes).XPos;
                 //AStarPath = autodrive.trajectorySequenceBuilder(autodrive.getPoseEstimate()).lineTo(new Vector2d(20,20)).lineTo(new Vector2d(30,0)).build();
                autodrive.followTrajectorySequenceAsync(AStarPath);
                pathGenerated=true;
            }
            if (!autodrive.isBusy()){
                graphMode=false;
                pathGenerated=false;
            }
        }

    }
    public static void resetGraph(){
        allNodes.clear();
        generateGraph();
    }
    public static void generateGraph() {

        //backstage nodes
        Node centerBack = new Node(30,0);
        Node BridgeBackBL = new Node(13,60);
        Node BridgeBackBR = new Node(13,36);
        Node BridgeBackCL = new Node(13,12);
        Node BridgeBackCR = new Node(13,-12);
        Node BridgeBackRL = new Node(13,-36);
        Node BridgeBackRR = new Node(13,-60);

        //front side nodes
        Node centerFront = new Node(-50,0);
        Node BridgeFrontBL = new Node(-37,60);
        Node BridgeFrontBR = new Node(-37,36);
        Node BridgeFrontCL = new Node(-37,12);
        Node BridgeFrontCR = new Node(-37,-12);
        Node BridgeFrontRL = new Node(-37,-36);
        Node BridgeFrontRR = new Node(-37,-60);


        //addition of all common nodes to the allNodes list
        allNodes.add(centerBack);

        allNodes.add(BridgeBackBL);
        allNodes.add(BridgeBackBR);
        allNodes.add(BridgeBackCL);
        allNodes.add(BridgeBackCR);
        allNodes.add(BridgeBackRL);
        allNodes.add(BridgeBackRR);

        allNodes.add(centerFront);

        allNodes.add(BridgeFrontBL);
        allNodes.add(BridgeFrontBR);
        allNodes.add(BridgeFrontCL);
        allNodes.add(BridgeFrontCR);
        allNodes.add(BridgeFrontRL);
        allNodes.add(BridgeFrontRR);



        //ADD EXCLUSIVE BLUE SIDE CONNECTIONS AND NODES
        Node BackdropB = new Node(48,36);
        Node behindBBack = new Node(36,36);
        Node collectBlue = new Node(-60,-64);
        Node BehindBCollect = new Node(-50,-64);
        allNodes.add(behindBBack);
        allNodes.add(BehindBCollect);
        allNodes.add(BackdropB);
        allNodes.add(collectBlue);

        BackdropB.addBranch(behindBBack);

        behindBBack.addBranch(centerBack);
        behindBBack.addBranch(BridgeBackBL);
        behindBBack.addBranch(BridgeBackBR);
        behindBBack.addBranch(BridgeBackCL);

        collectBlue.addBranch(BehindBCollect);

        BehindBCollect.addBranch(centerFront);
        BehindBCollect.addBranch(BridgeFrontCR);
        BehindBCollect.addBranch(BridgeFrontRL);
        BehindBCollect.addBranch(BridgeFrontRR);



        //ADD EXCLUSIVE RED SIDE CONNECTIONS AND NODES
        Node BackdropR = new Node(48,-36);
        Node behindRBack = new Node(36,-36);
        Node collectRed = new Node(-60,64);
        Node BehindRCollect = new Node(-50,64);


        //allNodes.add(behindRBack);
        //allNodes.add(BehindRCollect);
        //allNodes.add(BackdropR);
        //allNodes.add(collectRed);

        //BackdropR.addBranch(behindRBack);
        /*
        behindRBack.addBranch(centerBack);
        behindRBack.addBranch(BridgeBackCR);
        behindRBack.addBranch(BridgeBackRL);
        behindRBack.addBranch(BridgeBackRR);
        /**/
        //collectRed.addBranch(BehindRCollect);
        /*
        BehindRCollect.addBranch(centerFront);
        BehindRCollect.addBranch(BridgeFrontCL);
        BehindRCollect.addBranch(BridgeFrontBL);
        BehindRCollect.addBranch(BridgeFrontBR);
        */


        //MAKE THE COMMON CONNECTIONS (EDGES BETWEEN VERTEXES)
        centerBack.addBranch(BridgeBackBL);
        centerBack.addBranch(BridgeBackBR);
        centerBack.addBranch(BridgeBackCL);
        centerBack.addBranch(BridgeBackCR);
        centerBack.addBranch(BridgeBackRL);
        centerBack.addBranch(BridgeBackRR);

        BridgeBackBL.addBranch(BridgeFrontBL);
        BridgeBackBR.addBranch(BridgeFrontBR);

        BridgeBackCL.addBranch(BridgeFrontCL);
        BridgeBackCL.addBranch(BridgeFrontCR);
        BridgeBackCR.addBranch(BridgeFrontCL);
        BridgeBackCR.addBranch(BridgeFrontCR);

        BridgeBackRL.addBranch(BridgeFrontRL);
        BridgeBackRR.addBranch(BridgeFrontRR);

        centerFront.addBranch(BridgeFrontBL);
        centerFront.addBranch(BridgeFrontBR);
        centerFront.addBranch(BridgeFrontCL);
        centerFront.addBranch(BridgeFrontCR);
        centerFront.addBranch(BridgeFrontRL);
        centerFront.addBranch(BridgeFrontRR);
    }
    private void adicionarTelemetria(){
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
        telemetry.addData("raw TR",gamepad1.right_stick_x);
        /**/
        /*
        telemetry.addData("1",fL);
        telemetry.addData("2",fR);
        telemetry.addData("3",bL);
        telemetry.addData("4",bR);
        //telemetry.addData("multiplier",maximo);//garante que a velocidade maxima seja 1/**/
        /**/


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
        telemetry.addData("wristpos",Wrist.getPosition());
        telemetry.addData("artpos",Articulation.getPosition());


        telemetry.addData("goalxpos",finalgoalbraco);

        telemetry.addData("goalxpos",goalbraco);
        telemetry.addLine(String.valueOf(graphMode));
        telemetry.addLine(String.valueOf(pathGenerated));
        telemetry.update(); // adiciona tudo da telemetria
    }
    private void braco() {
        //atras = 1700
        //frente = 700
        //baixado = 0
        //kp = 0.01
        //PVar = 6


        if (gamepad2.dpad_down){
            finalgoalbraco =startingArmPos;
        }
        if (gamepad2.dpad_up){
            finalgoalbraco = MAXHEIGHT + startingArmPos;
        }
        //Articulation.setPosition(gamepad2.right_stick_x);
        //Arm2.setPosition(gamepad2.left_stick_x);
        if (gamepad2.x){
            Articulation.setPosition(0.15-0.15);
            Wrist.setPosition(0.35);
            //0.72 extend
            //0.35 gradado
        }
        if (gamepad2.dpad_left){
            Articulation.setPosition(0.5);
            Wrist.setPosition(0.05);

        }
        if (gamepad2.dpad_right){
            Articulation.setPosition(0.53-gamepad2.right_trigger*0.4);
            Wrist.setPosition(0.35+gamepad2.right_trigger*0.3);
            //0.35
        }

        //goalbraco+= GPVARIATION*(finalgoalbraco-goalbraco);
        goalbraco+= PVARIATION*(finalgoalbraco-goalbraco);
        if (((goalbraco > finalgoalbraco - LVARIATION) && (goalbraco < finalgoalbraco + LVARIATION))) {
            goalbraco = finalgoalbraco;
        } else {
            goalbraco+= PVARIATION*(finalgoalbraco-goalbraco);
        }


        //goalbraco = finalgoalbraco;
        /*
        if (!((goalbraco > finalgoalbraco - GPVARIATION) && (goalbraco < finalgoalbraco + GPVARIATION))) {
            if (goalbraco > finalgoalbraco) {
                goalbraco -= GPVARIATION;
            } else {
                goalbraco += GPVARIATION;
            }
        } else {
            goalbraco = finalgoalbraco;
        }
        */
        powerbraco = pid_braco.CalculatePID(mainArm.getCurrentPosition(), goalbraco, false);
        if (gamepad2.y) {
            mainArm.setPower(-0.1);
            finalgoalbraco = mainArm.getCurrentPosition();
            goalbraco = finalgoalbraco;
        } else {
            mainArm.setPower(powerbraco);
        }

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
        LClaw.setPosition(0.45);
    }
    private void openLeftClaw(){
        LClaw.setPosition(0.65);
    }
    private void closeRightClaw(){
        RClaw.setPosition(0.65);
    }
    private void openRightClaw(){
        RClaw.setPosition(0.45);
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
