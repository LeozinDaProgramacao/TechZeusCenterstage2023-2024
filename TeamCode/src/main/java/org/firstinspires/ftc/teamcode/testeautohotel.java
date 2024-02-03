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
@Autonomous(name="testeautohotel", group="BlueAuto")
public class testeautohotel extends LinearOpMode {






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

    BlueDetector.Location registred;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        Articulation = hardwareMap.get(Servo.class,"Articulation");
        LClaw = hardwareMap.get(Servo.class,"LClaw");
        RClaw = hardwareMap.get(Servo.class,"RClaw");
        Wrist = hardwareMap.get(Servo.class,"Wrist");
        mainArm = hardwareMap.get(DcMotor.class, "MainArm");


        Pose2d startPose = new Pose2d(0, 0, 0);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        /*
        //This trajectory sequence moves the robot to the middle of the square adjacent to all three lines

         */

        TrajectorySequence leftSequence= drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(2,2,0))
                .lineToConstantHeading(new Vector2d(3,-50))
                .build();




        waitForStart();
        //select specific trajectory for debugging
        drive.followTrajectorySequenceAsync(leftSequence);

        //alterna entre programações com base na detecção da câmera
        while (opModeIsActive()){
            drive.update();
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

