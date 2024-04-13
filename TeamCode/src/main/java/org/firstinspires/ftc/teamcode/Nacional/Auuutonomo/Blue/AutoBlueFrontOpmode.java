package org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Blue;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Blue.BlueFront.BlueFrontLeft;
import org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Blue.BlueFront.BlueFrontMid;
import org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Blue.BlueFront.BlueFrontRight;

import org.firstinspires.ftc.teamcode.Nacional.SubSystems.ArmMovement;
import org.firstinspires.ftc.teamcode.Nacional.SubSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.oldcode.BlueDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="BlueFrontYayy!♿", group="BlueAuto")
public class AutoBlueFrontOpmode extends LinearOpMode {

    BlueDetector.Location registred;

    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(12, 63-0.5, Math.toRadians(90));
        PoseStorage.currentPose = startPose;
        RobotHardware.setHardwareMap(hardwareMap);
        RobotHardware.initAll(1);
        RobotHardware.startup();
        ArmMovement.PVARIATION = 0.022;
        ArmMovement.LVARIATION = 500;


        RobotHardware.AutonomousDrive.setPoseEstimate(startPose);

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



        ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.CLOSED);
        ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.CLOSED);

        RobotHardware.moveArm(-0.2);
        waitForStart();
        RobotHardware.mainArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotHardware.mainArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RobotHardware.moveArm(0);


        //a câmera detecta em qual posição está o objeto de jogo
        registred = detector.getLocation();

        //camera é desligada para poupar recursos
        webcam.stopStreaming();

        //alterna entre programações com base na detecção da câmera
        switch (registred) {
            case LEFT:
                //posição esquerda
                RobotHardware.AutonomousDrive.followTrajectorySequenceAsync(BlueFrontLeft.getLeftSequence(RobotHardware.AutonomousDrive,startPose));
                break;
            case MIDDLE:
                //posição central
                RobotHardware.AutonomousDrive.followTrajectorySequenceAsync(BlueFrontMid.getMidSequence(RobotHardware.AutonomousDrive,startPose));
                break;
            case RIGHT:
                //posição direita
                RobotHardware.AutonomousDrive.followTrajectorySequenceAsync(BlueFrontRight.getRightSequence(RobotHardware.AutonomousDrive,startPose));
                break;
        }

        while (opModeIsActive()){
            RobotHardware.AutonomousDrive.update();
            ArmMovement.armPIDLoop(false);
            ArmMovement.AutoCloseClawSensor();
        }

        PoseStorage.currentPose = RobotHardware.AutonomousDrive.getPoseEstimate();
    }
}


