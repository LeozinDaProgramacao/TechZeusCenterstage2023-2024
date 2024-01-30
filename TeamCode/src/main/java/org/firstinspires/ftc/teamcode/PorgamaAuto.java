package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Autonomous(name="porgamauto", group="BlueAuto")
public class PorgamaAuto extends LinearOpMode {






    double armpos;
    double powerbraco;
    double finalgoalbraco=0;
    double goalbraco =0;
    public static double GPVARIATION=3;
    OpenCvWebcam webcam;

    private  Servo Articulation;
    private Servo LClaw;
    private Servo RClaw;
    private Servo Wrist;
    private CRServo mainArm;
    private Encoder armEncoder;
    PID pid_braco = new PID(0.01,0,0,0);

    BlueDetector.Location registred;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        Articulation = hardwareMap.get(Servo.class,"Articulation");
        LClaw = hardwareMap.get(Servo.class,"LClaw");
        RClaw = hardwareMap.get(Servo.class,"RClaw");
        Wrist = hardwareMap.get(Servo.class,"Wrist");
        mainArm = hardwareMap.get(CRServo.class, "BRACO_TEMP_MUDAR_DPS");
        armEncoder = new Encoder(hardwareMap.get(DcMotorEx.class,"BRmotor"));

        Pose2d startPose = new Pose2d(-36, 63, Math.toRadians(90));

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        BlueDetector detector = new BlueDetector(telemetry);
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

        /*TrajectorySequence leftSequence= drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-39,32,0))
                .addDisplacementMarker(()->
                {
                    Articulation.setPosition(0.6-0.4);
                    Wrist.setPosition(0.37+0.35);

                })
                .waitSeconds(0.2)
                .addDisplacementMarker(()->
                {
                    openLeftClaw();
                })
                //.lineToLinearHeading(new Pose2d(-54.5,25.5-12,Math.toRadians(180)))
                //moves to get the  extra pixel
                .lineToLinearHeading(new Pose2d(-56,13,Math.toRadians(180)))
                .addDisplacementMarker(()->{
                    //elevates claw to get single pixel from stack
                    Wrist.setPosition(0.51);
                    Articulation.setPosition(0.45);
                })
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(20,11,Math.toRadians(180)),Math.toRadians(180))
                .addDisplacementMarker(()->
                {
                    armpos = 1700;
                    Articulation.setPosition(0.57);
                    Wrist.setPosition(0.15);
                })
                //.splineToConstantHeading(new Vector2d(45,36),0)
                .lineTo(new Vector2d(46,45))//thi is extra to test
                .forward(4)
                .addDisplacementMarker(()->{
                    openLeftClaw();
                    openRightClaw();
                })
                .build();
        */
        TrajectorySequence mideSequence= drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    Articulation.setPosition(0.55);
                    Wrist.setPosition(0.45);
                })
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-56.5,36.2,Math.toRadians(200)),Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    closeRightClaw();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    Wrist.setPosition(0.2);
                })
                .build();



        /*TrajectorySequence rightSequence= drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-46,13,Math.toRadians(90)))
                .addDisplacementMarker(()->
                {
                    Articulation.setPosition(0.6-0.4);
                    Wrist.setPosition(0.37+0.35);

                })
                .waitSeconds(0.2)
                .addDisplacementMarker(()->
                {
                    openLeftClaw();
                    Articulation.setPosition(0.15);
                    Wrist.setPosition(0.35);
                })
                .lineToLinearHeading(new Pose2d(-56,14,Math.toRadians(180)))
                .addDisplacementMarker(()->{
                    Articulation.setPosition(0.6-0.4);
                    Wrist.setPosition(0.37+0.35);
                })
                .waitSeconds(0.5)
                .addDisplacementMarker(()->
                {
                    closeLeftClaw();
                })
                .waitSeconds(0.2)
                .addDisplacementMarker(()->
                {
                    openLeftClaw();
                    Articulation.setPosition(0.15);
                    Wrist.setPosition(0.35);
                })
                .setReversed(true)
                //.waitSeconds(1)
                .splineToLinearHeading(new Pose2d(20,11,Math.toRadians(180)),Math.toRadians(0))
                .addDisplacementMarker(()->
                {
                    armpos = 1700;
                    Articulation.setPosition(0.57);
                    Wrist.setPosition(0.15);
                })
                //.splineToConstantHeading(new Vector2d(45,36),0)
                .lineTo(new Vector2d(46,29))
                .forward(4)
                .addDisplacementMarker(()->{
                    openLeftClaw();
                    openRightClaw();
                })
                //thi is extra to test
                .build();
                */


        closeLeftClaw();
        closeRightClaw();
        mainArm.setPower(-0.15);
        waitForStart();
        double startEncoderPos = armEncoder.getCurrentPosition();
        drive.followTrajectorySequenceAsync(mideSequence);
        closeLeftClaw();
        closeRightClaw();
        //a câmera detecta em qual posição está o objeto de jogo
        registred = detector.getLocation();
        webcam.stopStreaming();

        //alterna entre programações com base na detecção da câmera
        /*
        switch (registred) {
            case LEFT:
                //posição esquerda
                drive.followTrajectorySequenceAsync(leftSequence);
                break;
            case MIDDLE:
                //posição central
                drive.followTrajectorySequenceAsync(midSequence);
                break;
            case RIGHT:
                //posição direita
                drive.followTrajectorySequenceAsync(rightSequence);
                break;
        }*/
        openLeftClaw();
        openRightClaw();
        while (opModeIsActive()){
            drive.update();
            //braco();
        }

    }





    private void braco() {
        //atras = 1700
        //frente = 700
        //baixado = 0
        //kp = 0.01
        //PVar = 6


        if (!((goalbraco > finalgoalbraco - GPVARIATION) && (goalbraco < finalgoalbraco + GPVARIATION))) {
            if (goalbraco > finalgoalbraco) {
                goalbraco -= GPVARIATION;
            } else {
                goalbraco += GPVARIATION;
            }
        } else {
            goalbraco = finalgoalbraco;
        }
        powerbraco = pid_braco.CalculatePID(armEncoder.getCurrentPosition(), goalbraco, false);
        mainArm.setPower(powerbraco);


    }
    private void closeLeftClaw(){
        LClaw.setPosition(0.4);
    }
    private void openLeftClaw(){
        LClaw.setPosition(0.7);
    }
    private void closeRightClaw(){
        RClaw.setPosition(0.7);
    }
    private void openRightClaw(){
        RClaw.setPosition(0.4);
    }
}

