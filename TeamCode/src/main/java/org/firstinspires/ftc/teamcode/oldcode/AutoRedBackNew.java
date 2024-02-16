package org.firstinspires.ftc.teamcode.oldcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Autonomous(name="AutoRedBackNew", group="RedAuto")
@Disabled
public class AutoRedBackNew extends LinearOpMode {
    double armpos;
    double powerbraco;
    double finalgoalbraco=0;
    double goalbraco =0;
    public static double PVARIATION=0.015;
    public static int LVARIATION = 100;
    public static int MAXHEIGHT = 1850;
    OpenCvWebcam webcam;

    private  Servo Articulation;
    private Servo LClaw;
    private Servo RClaw;
    private Servo Wrist;
    private DcMotor mainArm;
    PID pid_braco = new PID(0.008,0,0,0);

    RedDetector.Location registred;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        Articulation = hardwareMap.get(Servo.class,"Articulation");
        LClaw = hardwareMap.get(Servo.class,"LClaw");
        RClaw = hardwareMap.get(Servo.class,"RClaw");
        Wrist = hardwareMap.get(Servo.class,"Wrist");
        mainArm = hardwareMap.get(DcMotor.class, "MainArm");


        Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(-90));

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        RedDetector detector = new RedDetector(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        /*
        //This trajectory sequence moves the robot to the middle of the square adjacent to all three lines

         */

        TrajectorySequence rightSequence= drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    Articulation.setPosition(0.35);
                    //Wrist.setPosition(0.5);
                    Wrist.setPosition(0.465);
                })
                .lineToLinearHeading(new Pose2d(-40,-31,0))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openRightClaw();
                })
                .setReversed(true)

                .back(3)
                .splineToLinearHeading(new Pose2d(-53,-16,Math.toRadians(180)),Math.toRadians(180))
                .forward(3.5)

                //moves to get the  extra pixel
                .UNSTABLE_addDisplacementMarkerOffset(0,()-> {
                    closeRightClaw();
                })
                .back(3)
                .splineToConstantHeading(new Vector2d(-48,-10),0)
                //.lineToLinearHeading(new Pose2d(-54.5,25.5-12,Math.toRadians(180)))
                //.waitSeconds(1)
                .splineToLinearHeading(new Pose2d(20,-8,Math.toRadians(180)),Math.toRadians(0))
                .UNSTABLE_addDisplacementMarkerOffset(-10,()->
                {
                    finalgoalbraco = MAXHEIGHT;
                    Articulation.setPosition(0.252);
                    Wrist.setPosition(0.251);
                })
                //.splineToConstantHeading(new Vector2d(45,36),0)

                //moves to backdrop to deposit pixel
                .splineToConstantHeading(new Vector2d(49,-40),0)
                .waitSeconds(0.5)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openRightClaw();
                    openLeftClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    finalgoalbraco=0;
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(40,-24),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(58,-10),Math.toRadians(-20))
                .build();



        TrajectorySequence mideSequence= drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    Articulation.setPosition(0.35);
                    //Wrist.setPosition(0.5);
                    Wrist.setPosition(0.465);
                })
                .lineToLinearHeading(new Pose2d(-39,-8,Math.toRadians(-90)))

                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openRightClaw();
                })
                .waitSeconds(0.2)
                .turn(Math.toRadians(-90))
                //moves to get the  extra pixel
                .splineToLinearHeading(new Pose2d(-53,-16,Math.toRadians(180)),Math.toRadians(180))
                .forward(3.5)
                .UNSTABLE_addDisplacementMarkerOffset(0,()-> {
                    closeRightClaw();
                })
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-48,-10),0)
                //.lineToLinearHeading(new Pose2d(-54.5,25.5-12,Math.toRadians(180)))
                //.waitSeconds(1)
                .splineToLinearHeading(new Pose2d(20,-8,Math.toRadians(180)),Math.toRadians(0))
                .UNSTABLE_addDisplacementMarkerOffset(-10,()->
                {
                    finalgoalbraco = MAXHEIGHT;
                    Articulation.setPosition(0.252);
                    Wrist.setPosition(0.251);
                })
                //.splineToConstantHeading(new Vector2d(45,36),0)
                .splineToConstantHeading(new Vector2d(49,-33),0)
                .waitSeconds(0.5)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openRightClaw();
                    openLeftClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    finalgoalbraco=0;
                })
                .setReversed(false)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(40,-24),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(58,-10),Math.toRadians(-20))
                .build();

        TrajectorySequence leftSequence= drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    Articulation.setPosition(0.35);
                    //Wrist.setPosition(0.5);
                    Wrist.setPosition(0.465);
                })
                .lineToLinearHeading(new Pose2d(-45.5,-10,Math.toRadians(-90)))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openRightClaw();
                })
                .turn(Math.toRadians(-90))
                //moves to get the  extra pixel
                .splineToLinearHeading(new Pose2d(-53,-16,Math.toRadians(180)),Math.toRadians(180))
                .forward(3.5)
                .UNSTABLE_addDisplacementMarkerOffset(0,()-> {
                    closeRightClaw();
                })
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-48,-10),0)
                //v.lineToLinearHeading(new Pose2d(-54.5,25.5-12,Math.toRadians(180)))
                //.waitSeconds(1)
                .splineToLinearHeading(new Pose2d(20,-8,Math.toRadians(180)),Math.toRadians(0))
                .UNSTABLE_addDisplacementMarkerOffset(-10,()->
                {
                    finalgoalbraco = MAXHEIGHT;
                    Articulation.setPosition(0.252);
                    Wrist.setPosition(0.251);
                })
                //.splineToConstantHeading(new Vector2d(45,36),0)
                .splineToConstantHeading(new Vector2d(49,-29.5),0)
                .waitSeconds(0.5)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openRightClaw();
                    openLeftClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    finalgoalbraco=0;
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(40,-24),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(58,-10),Math.toRadians(20))
                .build();



        closeRightClaw();
        closeRightClaw();

        mainArm.setPower(-0.2);

        waitForStart();
        mainArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //select specific trajectory for debugging
        //drive.followTrajectorySequenceAsync(leftSequence);
        closeLeftClaw();
        closeRightClaw();
        //a câmera detecta em qual posição está o objeto de jogo
        registred = detector.getLocation();
        webcam.stopStreaming();

        //alterna entre programações com base na detecção da câmera

        switch (registred) {
            case LEFT:
                //posição esquerda
                drive.followTrajectorySequenceAsync(leftSequence);
                break;
            case MIDDLE:
                //posição central
                drive.followTrajectorySequenceAsync(mideSequence);
                break;
            case RIGHT:
                //posição direita
                drive.followTrajectorySequenceAsync(rightSequence);
                break;
        }
        closeLeftClaw();
        closeRightClaw();
        while (opModeIsActive()){
            drive.update();
            braco();
        }
        PoseStorage.currentPose = drive.getPoseEstimate();

    }





    private void braco() {
        //atras = 1850
        //frente = 700
        //baixado = 0
        //kp = 0.01
        //PVar = 6

        /*
        if (gamepad2.dpad_down){
            finalgoalbraco = 0;
        }
        if (gamepad2.dpad_up){
            finalgoalbraco = 1850;
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
            Articulation.setPosition(0.57);
            Wrist.setPosition(0.05);

        }
        if (gamepad2.dpad_right){
            Articulation.setPosition(0.6-0.4);
            Wrist.setPosition(0.37+0.35);
            //0.35
        }*/

        goalbraco+= PVARIATION*(finalgoalbraco-goalbraco);
        if (((goalbraco > finalgoalbraco - LVARIATION) && (goalbraco < finalgoalbraco + LVARIATION))) {
            goalbraco = finalgoalbraco;
        } else {
            goalbraco+= PVARIATION*(finalgoalbraco-goalbraco);
        }
        powerbraco = pid_braco.CalculatePID(mainArm.getCurrentPosition(), goalbraco, false);
        mainArm.setPower(powerbraco);


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
}

