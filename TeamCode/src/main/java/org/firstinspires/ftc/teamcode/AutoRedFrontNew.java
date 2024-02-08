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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Autonomous(name="AutoRedFrontNew", group="RedAuto")
public class AutoRedFrontNew extends LinearOpMode {






    double armpos;
    double powerbraco;
    double finalgoalbraco=0;
    double goalbraco =0;
    public static double PVARIATION=0.015;
    public static int LVARIATION = 100;
    public static int MAXHEIGHT = 1800;

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


        Pose2d startPose = new Pose2d(12, -63, Math.toRadians(-90));

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

        TrajectorySequence leftSequencee= drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    Articulation.setPosition(0.35);
                    //Wrist.setPosition(0.5);
                    Wrist.setPosition(0.45);
                })
                .lineToLinearHeading(new Pose2d(36,-32,Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openRightClaw();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    finalgoalbraco = MAXHEIGHT;
                    Articulation.setPosition(0.252);
                    Wrist.setPosition(0.3);
                })
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(49,-39),0)
                .waitSeconds(0.1)
                .back(6)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openLeftClaw();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    finalgoalbraco=0;
                })
                .splineToConstantHeading(new Vector2d(13,-60),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30,-60),Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    Articulation.setPosition(0.35);
                    //Wrist.setPosition(0.5);
                    Wrist.setPosition(0.48);
                })
                .splineToLinearHeading(new Pose2d(-54,-38,Math.toRadians(180)),Math.toRadians(180))


                .waitSeconds(0.2)
                .forward(4)
                //.setReversed(true)
                //.back(3)
                //.splineToLinearHeading(new Pose2d(-56,12,Math.toRadians(180)),Math.toRadians(180))

                //moves to get the  extra pixel

                .UNSTABLE_addTemporalMarkerOffset(0,()-> {
                    closeRightClaw();
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    Wrist.setPosition(0.2);
                })
                .back(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-30,-60,Math.toRadians(180)),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(13,-60),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    finalgoalbraco = MAXHEIGHT;
                    Articulation.setPosition(0.252);
                    Wrist.setPosition(0.251);
                })
                .splineToConstantHeading(new Vector2d(47,-39.5),0)
                .waitSeconds(0.3)
                .back(8)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openRightClaw();
                }).waitSeconds(0.3)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    finalgoalbraco=0;
                })
                .forward(5)
                .splineToConstantHeading(new Vector2d(40,-60),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60,-60),Math.toRadians(0))
                .build();



        TrajectorySequence mideSequence= drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    Articulation.setPosition(0.252);
                    //Wrist.setPosition(0.5);
                    Wrist.setPosition(0.45);
                })
                .lineToLinearHeading(new Pose2d(12,-38.5,Math.toRadians(90-0.01)))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openRightClaw();
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    finalgoalbraco = MAXHEIGHT;
                    Articulation.setPosition(0.252);
                    Wrist.setPosition(0.3);
                })
                .setReversed(true)

                //move to place on backdrop
                .splineToLinearHeading(new Pose2d(49,-33.5,Math.toRadians(180)),0)
                .waitSeconds(0.2)
                .back(6)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openLeftClaw();
                })






                .waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    finalgoalbraco=0;
                })
                .splineToConstantHeading(new Vector2d(13,-60),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30,-60),Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    Articulation.setPosition(0.35);
                    //Wrist.setPosition(0.5);
                    Wrist.setPosition(0.48);
                })
                .splineToLinearHeading(new Pose2d(-54,-38,Math.toRadians(180)),Math.toRadians(180))


                .waitSeconds(0.2)
                .forward(4)
                //.setReversed(true)
                //.back(3)
                //.splineToLinearHeading(new Pose2d(-56,12,Math.toRadians(180)),Math.toRadians(180))

                //moves to get the  extra pixel

                .UNSTABLE_addTemporalMarkerOffset(0,()-> {
                    closeRightClaw();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    Wrist.setPosition(0.2);
                })
                .back(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-30,-60,Math.toRadians(180)),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(13,-60),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    finalgoalbraco = MAXHEIGHT;
                    Articulation.setPosition(0.252);
                    Wrist.setPosition(0.251);
                })
                .splineToConstantHeading(new Vector2d(47,-39),0)
                .waitSeconds(0.5)
                .back(8)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openRightClaw();
                }).waitSeconds(0.3)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    finalgoalbraco=0;
                })
                .forward(10)
                .splineToConstantHeading(new Vector2d(40,-60),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60,-60),Math.toRadians(0))
                .build();

        TrajectorySequence rightSequence= drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    Articulation.setPosition(0.35);
                    //Wrist.setPosition(0.5);
                    Wrist.setPosition(0.45);
                })
                .splineToLinearHeading(new Pose2d(14,-31,Math.toRadians(180)),Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openRightClaw();
                })
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    finalgoalbraco = MAXHEIGHT;
                    Articulation.setPosition(0.252);
                    Wrist.setPosition(0.3);
                })
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(49,-29.5,Math.toRadians(180)),0)
                .waitSeconds(0.2)
                .back(6)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openLeftClaw();
                })









                .waitSeconds(0.15)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    finalgoalbraco=0;
                })
                .splineToConstantHeading(new Vector2d(13,-60),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30,-60),Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    Articulation.setPosition(0.35);
                    //Wrist.setPosition(0.5);
                    Wrist.setPosition(0.48);
                })
                .splineToLinearHeading(new Pose2d(-54,-38,Math.toRadians(180)),Math.toRadians(180))


                .waitSeconds(0.2)
                .forward(4)
                //.setReversed(true)
                //.back(3)
                //.splineToLinearHeading(new Pose2d(-56,12,Math.toRadians(180)),Math.toRadians(180))

                //moves to get the  extra pixel

                .UNSTABLE_addTemporalMarkerOffset(0,()-> {
                    closeRightClaw();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    Wrist.setPosition(0.2);
                })
                .setReversed(true)
                .back(1)
                .splineToLinearHeading(new Pose2d(-30,-60,Math.toRadians(180)),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(13,-60),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    finalgoalbraco = MAXHEIGHT;
                    Articulation.setPosition(0.252);
                    Wrist.setPosition(0.251);
                })
                .splineToConstantHeading(new Vector2d(47,-39),0)
                .waitSeconds(0.3)
                .back(8)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openRightClaw();
                }).waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    finalgoalbraco=0;
                })
                .forward(10)
                .splineToConstantHeading(new Vector2d(40,-60),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60,-60),Math.toRadians(0))
                .build();
        /**/


        closeRightClaw();
        closeLeftClaw();

        mainArm.setPower(-0.2);
        waitForStart();
        mainArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mainArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //select specific trajectory for debugging
        drive.followTrajectorySequenceAsync(rightSequence);

        //a câmera detecta em qual posição está o objeto de jogo
        registred = detector.getLocation();
        webcam.stopStreaming();

        //alterna entre programações com base na detecção da câmera

        switch (registred) {
            case LEFT:
                //posição esquerda
                drive.followTrajectorySequenceAsync(rightSequence);
                break;
            case MIDDLE:
                //posição central
                drive.followTrajectorySequenceAsync(mideSequence);
                break;
            case RIGHT:
                //posição direita
                drive.followTrajectorySequenceAsync(leftSequencee);
                break;
        }

        closeLeftClaw();
        closeLeftClaw();
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
        LClaw.setPosition(0.35);
    }
    private void openLeftClaw(){
        LClaw.setPosition(0.65);
    }
    private void closeRightClaw(){
        RClaw.setPosition(0.65);
    }
    private void openRightClaw(){
        RClaw.setPosition(0.35);
    }
}

