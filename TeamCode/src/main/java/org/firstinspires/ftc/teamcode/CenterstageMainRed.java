package org.firstinspires.ftc.teamcode;



import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name = "CenterstageMainRed")

public class CenterstageMainRed extends LinearOpMode {
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
    private Servo PlaneServo;
    //private Servo servoArticulacao;
    //private Servo servoGarra;
    private IMU imu;
    YawPitchRollAngles orientation;
    double FeedFHang = 0;
    public static double PVARIATION=0.017;
    public static int LVARIATION = 100;
    public static double HANGVARIATION =6;
    public static int MAXHEIGHT = 1750;
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
    boolean graphMode = false;
    boolean pathGenerated = false;
    boolean yOn;
    double powerbraco;
    double finalgoalbraco = 0;
    double goalbraco;
    //PID pid_turn = new PID(0.5,0,0,0);
    PID pid_turn = new PID(0.3,0.0,0.0,0.0);
    //PID pid_turn = new PID(0.6,0.000,0.0,0.3);
    PID pid_braco = new PID(0.005,0,0,0);
    PID coreHexEsq = new PID(0.05,0,0,0);
    PID coreHexDir = new PID(0.05,0,0,0);

    simpleSwitch leftClawSwitch = new simpleSwitch();
    simpleSwitch rightClawSwitch = new simpleSwitch();
    double prevtime=0;
    double Artpos =0;
    public static List<Node> allNodes= new ArrayList<>();
    double goalX=0;
    private AprilTagProcessor aprilTag;
    double CAM_DIST_TO_CENTER = 6.5;
    double actualHead;
    double xtrans;
    double ytrans;
    private VisionPortal visionPortal;
    SampleMecanumDriveCancelable autodrive;
    private double finalgoalhang=0;
    private double goalhang=0;

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag
        );

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    private Pose2d LocateWithAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        if (currentDetections.size()>0) {
            double avgX = 0;
            double avgY = 0;
            double avgHead = 0;
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    double xtag = 0;
                    double ytag = 0;
                    switch (detection.id) {
                        case 1:
                            xtag = 63;
                            ytag = 41.5;
                            break;
                        case 2:
                            xtag = 63;
                            ytag = 35.5;
                            break;
                        case 3:
                            xtag = 63;
                            ytag = 29.5;
                            break;
                        case 4:
                            xtag = 63;
                            ytag= -29.5;
                            break;
                        case 5:
                            xtag = 63;
                            ytag =-35.5;
                            break;
                        case 6:
                            xtag = 63;
                            ytag = -41.5;
                            break;
                    }
                    actualHead = -(detection.ftcPose.yaw - detection.ftcPose.bearing);
                    ytrans = Math.sin(Math.toRadians(actualHead)) * detection.ftcPose.range;
                    xtrans = Math.cos(Math.toRadians(actualHead)) * detection.ftcPose.range;

                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    //telemetry.addLine(String.format("Xtrans %6.1f Ytrans %6.1f",xtrans,ytrans));
                    telemetry.addLine(String.format("CAM FIELD XY %6.1f %6.1f", xtag - xtrans, ytag - ytrans, detection.ftcPose.bearing));
                    telemetry.addLine(String.format("BOT FIELD XY HEAD %6.1f %6.1f %6.1f",
                            (xtag - xtrans) - (Math.cos(Math.toRadians(-detection.ftcPose.yaw)) * CAM_DIST_TO_CENTER),
                            (ytag - ytrans) - (Math.sin(Math.toRadians(-detection.ftcPose.yaw)) * CAM_DIST_TO_CENTER),
                            detection.ftcPose.yaw));
                    //telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    avgX+= (xtag - xtrans) - (Math.cos(Math.toRadians(-detection.ftcPose.yaw)) * CAM_DIST_TO_CENTER);
                    avgY += (ytag - ytrans) - (Math.sin(Math.toRadians(-detection.ftcPose.yaw)) * CAM_DIST_TO_CENTER);
                    avgHead += detection.ftcPose.yaw+180;
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            // Add "key" information to telemetry
            //telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            //telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            //telemetry.addLine("RBE = Range, Bearing & Elevation");
            return new Pose2d(avgX/currentDetections.size(), avgY/currentDetections.size(), -Math.toRadians(avgHead/currentDetections.size()));
        } else {
            return autodrive.getPoseEstimate();
        }
    }   // end method telemetryAprilTag()

    /**
     * This function is executed when the opmode is executed
     */
    @Override
    public void runOpMode() {

        //inicializa hardware
        imu = hardwareMap.get(IMU.class, "imu");
        frontRight = hardwareMap.get(DcMotor.class, "FRmotor");
        backRight = hardwareMap.get(DcMotor.class, "BRmotor");
        frontLeft = hardwareMap.get(DcMotor.class, "FLmotor");
        backLeft = hardwareMap.get(DcMotor.class, "BLmotor");
        mainArm = hardwareMap.get(DcMotor.class, "MainArm");
        CoreEsq= hardwareMap.get(DcMotor.class, "Barra1");
        CoreDir = hardwareMap.get(DcMotor.class, "Barra2");


        Articulation = hardwareMap.get(Servo.class,"Articulation");
        LClaw = hardwareMap.get(Servo.class,"LClaw");
        RClaw = hardwareMap.get(Servo.class,"RClaw");
        Wrist = hardwareMap.get(Servo.class,"Wrist");
        PlaneServo = hardwareMap.get(Servo.class,"PlaneServo");

        CoreEsq.setDirection(DcMotorSimple.Direction.FORWARD);
        CoreDir.setDirection(DcMotorSimple.Direction.REVERSE);

        CoreDir.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CoreEsq.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        CoreDir.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        CoreEsq.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        //corrige orientação dos motores
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

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


        //inicializa o robô para o teleop


        mainArm.setPower(-0.2);
        startingArmPos = mainArm.getCurrentPosition();
        PlaneServo.setPosition(0);
        initAprilTag();

        waitForStart();

        mainArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mainArm.setPower(0);

        double CurrentFront = 0;
        autodrive = new SampleMecanumDriveCancelable(hardwareMap);
        autodrive.setPoseEstimate(new Pose2d(24,0,0));

        //grafo é gerado para uso no teleop
        generateGraph();

        while  (opModeIsActive()) {
            braco();
            inputs();
            moveBase();
            adicionarTelemetria();
            manageWebcam();

        }
    }

    /**
     * Manages webcam for the dettection of april tags and conservation of memmory, pressing X engages webcam and pressing B turns it off
     * Gerencia a câmera para o teleop, apertar X liga a câmera e B desliga
     */
    private void manageWebcam() {
        if (gamepad1.x){
            visionPortal.resumeStreaming();
        } else if (gamepad1.b){
            visionPortal.stopStreaming();
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

    /**
     * Gerencia o IMU e o PID da base, evitando curvas não intencionais e proporcionando a orientação
     * nescessaria para o controle orientado ao controlador
     */
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

    /**
     * Movimenta a base, nesta função há dois modos de ação, o primeiro é o controle manual
     * o segundo é o modo de movimentação por grafos, que pode ser habilidado ao pressionar o
     * dpad esquerdo, fazendo o robô gerar uma nova rota e começar a segui-la
     */
    private void moveBase(){
        autodrive.update();
        if (gamepad1.dpad_right){
            graphMode=true;
        }

        if (!graphMode||gamepad1.dpad_left) {
            if (gamepad1.dpad_left){
                autodrive.breakFollowing();
                turn = 0.002;
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
            telemetry.addData("detects",aprilTag.getDetections().size());
        } else{
            if (!pathGenerated){

                autodrive.setPoseEstimate(LocateWithAprilTag());
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
                turn = 0.002;
                manageIMU();
            }
        }

    }

    /**
     * Esta função reseta as funcionalidades nescessarias para o uso do movimento automatizado usando grafos
     */
    public static void resetGraph(){
        allNodes.clear();
        generateGraph();
    }

    /**
     * Gera o grafo do sistema de movimento automático, com os vértices e arestas tudinho :>
     */
    public static void generateGraph() {

        //backstage nodes
        Node centerBack = new Node(30,0);
        Node BridgeBackBL = new Node(13,60);
        Node BridgeBackBR = new Node(13,36);
        Node BridgeBackCL = new Node(13,11);
        Node BridgeBackCR = new Node(13,-11);
        Node BridgeBackRL = new Node(13,-36);
        Node BridgeBackRR = new Node(13,-60);

        //front side nodes
        Node centerFront = new Node(-50,0);
        Node BridgeFrontBL = new Node(-37,60);
        Node BridgeFrontBR = new Node(-37,36);
        Node BridgeFrontCL = new Node(-37,11);
        Node BridgeFrontCR = new Node(-37,-11);
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
        /*
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
        */


        //ADD EXCLUSIVE RED SIDE CONNECTIONS AND NODES
        Node BackdropR = new Node(48,-36);
        Node behindRBack = new Node(36,-36);
        Node collectRed = new Node(-60,64);
        Node BehindRCollect = new Node(-50,64);


        allNodes.add(behindRBack);
        allNodes.add(BehindRCollect);
        allNodes.add(BackdropR);
        allNodes.add(collectRed);

        BackdropR.addBranch(behindRBack);

        behindRBack.addBranch(centerBack);
        behindRBack.addBranch(BridgeBackCR);
        behindRBack.addBranch(BridgeBackRL);
        behindRBack.addBranch(BridgeBackRR);
        /**/
        collectRed.addBranch(BehindRCollect);

        BehindRCollect.addBranch(centerFront);
        BehindRCollect.addBranch(BridgeFrontCL);
        BehindRCollect.addBranch(BridgeFrontBL);
        BehindRCollect.addBranch(BridgeFrontBR);



        //MAKE THE COMMON CONNECTIONS (EDGES BETWEEN VERTEXES)
        centerBack.addBranch(BridgeBackBL);
        centerBack.addBranch(BridgeBackBR);
        centerBack.addBranch(BridgeBackCL);
        centerBack.addBranch(BridgeBackCR);
        centerBack.addBranch(BridgeBackRL);
        centerBack.addBranch(BridgeBackRR);

        BridgeBackBL.addBranch(BridgeFrontBL);
        BridgeBackBR.addBranch(BridgeFrontBR);


        //MAKE ONE DIRECTIONAL CONNECTIONS BECAUSE THE FUCKING ROBOT CANT FIT UNDER A BRIDGE
        BridgeFrontCL.addOneDirectionalBranch(BridgeBackCL);
        BridgeFrontCL.addOneDirectionalBranch(BridgeBackCR);
        BridgeFrontCR.addOneDirectionalBranch(BridgeBackCL);
        BridgeFrontCR.addOneDirectionalBranch(BridgeBackCR);

        BridgeBackRL.addBranch(BridgeFrontRL);
        BridgeBackRR.addBranch(BridgeFrontRR);

        centerFront.addBranch(BridgeFrontBL);
        centerFront.addBranch(BridgeFrontBR);
        centerFront.addBranch(BridgeFrontCL);
        centerFront.addBranch(BridgeFrontCR);
        centerFront.addBranch(BridgeFrontRL);
        centerFront.addBranch(BridgeFrontRR);
    }
    /**
     *Description:
     *this funtion adds all debugging data needed to the telemetry board
     * adiciona toda a telemetria
     */
    private void adicionarTelemetria(){


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

    /**
     * Esta função basicamente controla as responsabilidades do segundo controlador:
     * Controle do setpoint (alvo) do PID do Braço
     * Controle das posições da garra e da articulação
     * Uso do PID/FeedForward usado para pendurar nas travessas da arena
     */
    private void braco() {
        //atras = 1700
        //frente = 700
        //baixado = 0
        //kp = 0.01
        //PVar = 6
        if (gamepad1.left_trigger>0.5&&gamepad1.right_trigger>0.5){
            PlaneServo.setPosition(1);
        }
        if (gamepad2.right_trigger>0.5&&gamepad2.left_trigger>0.5){
            finalgoalhang = 250;
            FeedFHang = -0.05;
        } if (gamepad2.x&&gamepad2.y){
            finalgoalhang = 800;
            FeedFHang =0;
        } if (gamepad2.b){
            finalgoalhang=0;
            FeedFHang = 0;
        }
        //goalhang+= PVARIATION*(finalgoalhang-goalhang);
        if (((goalhang > finalgoalhang - HANGVARIATION) && (goalhang < finalgoalhang + HANGVARIATION))) {
            goalhang = finalgoalhang;
        } else if (goalhang>finalgoalhang){
            goalhang-=HANGVARIATION;
        } else if (goalhang<finalgoalhang){
            goalhang+=HANGVARIATION;
        }
        double powEsq= coreHexEsq.CalculatePID(CoreEsq.getCurrentPosition(),goalhang,false);
        double powDir =coreHexDir.CalculatePID(CoreDir.getCurrentPosition(),goalhang,false);
        CoreEsq.setPower(powEsq+FeedFHang);
        CoreDir.setPower(powDir+FeedFHang);
        telemetry.addData("left corehex",CoreEsq.getCurrentPosition());
        telemetry.addData("right corehex",CoreDir.getCurrentPosition());


        if (gamepad2.dpad_down){
            finalgoalbraco =startingArmPos;
        }
        if (gamepad2.dpad_up){
            finalgoalbraco = MAXHEIGHT + startingArmPos;
        }
        //Articulation.setPosition(gamepad2.right_stick_x);
        //Arm2.setPosition(gamepad2.left_stick_x);
        if (gamepad2.x){
            Articulation.setPosition(0.1);
            Wrist.setPosition(0.35);
            //0.72 extend
            //0.35 gradado
        }
        if (gamepad2.dpad_left){
            Articulation.setPosition(0.252);
            Wrist.setPosition(0.251);
        }
        if (gamepad2.dpad_right){
            Articulation.setPosition(0.53-gamepad2.right_trigger*0.4);
            Wrist.setPosition(0.35+gamepad2.right_trigger*0.3);
            //0.35
        }


        goalbraco+= PVARIATION*(finalgoalbraco-goalbraco);
        if (((goalbraco > finalgoalbraco - LVARIATION) && (goalbraco < finalgoalbraco + LVARIATION))) {
            goalbraco = finalgoalbraco;
        } else {
            goalbraco+= PVARIATION*(finalgoalbraco-goalbraco);
        }
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

    public double ConvertAngle(double angle){
        double return_angle = angle;
        if(angle>Math.PI){
            return_angle -= 2*Math.PI;
        }
        return return_angle;
    }

    /**
     * Essa classe permite usar os inputs booleanos de botôes dos controles para controlar objetos
     * que agem como interruptores, assim permitindo ligar e desligar funções com o mesmo botão
     */
    public static class simpleSwitch{
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
