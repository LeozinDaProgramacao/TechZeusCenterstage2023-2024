package org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Blue;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Blue.BlueBack.BlueBackLeft;
import org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Blue.BlueBack.BlueBackMid;
import org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Blue.BlueBack.BlueBackRight;
@Autonomous(name="BlueBackYipee!", group="BlueAuto")
public class AutoBlueBackOpmode extends LinearOpMode {

    BlueDetector.Location registred;

    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d startPose = new Pose2d(-36, 63, Math.toRadians(90));
        PoseStorage.currentPose = startPose;
        RobotHardware.setHardwareMap(hardwareMap);
        RobotHardware.initAll();
        RobotHardware.startup();

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

        waitForStart();

        //a câmera detecta em qual posição está o objeto de jogo
        registred = detector.getLocation();

        //camera é desligada para poupar recursos
        webcam.stopStreaming();

        //alterna entre programações com base na detecção da câmera
        switch (registred) {
            case LEFT:
                //posição esquerda
                RobotHardware.AutonomousDrive.followTrajectorySequenceAsync(BlueBackLeft.getLeftSequence(RobotHardware.AutonomousDrive,startPose));
                break;
            case MIDDLE:
                //posição central
                RobotHardware.AutonomousDrive.followTrajectorySequenceAsync(BlueBackMid.getMidSequence(RobotHardware.AutonomousDrive,startPose));
                break;
            case RIGHT:
                //posição direita
                RobotHardware.AutonomousDrive.followTrajectorySequenceAsync(BlueBackRight.getRightSequence(RobotHardware.AutonomousDrive,startPose));
                break;
        }

        while (opModeIsActive()){
            RobotHardware.AutonomousDrive.update();
            ArmMovement.armPIDLoop(false);
        }

        PoseStorage.currentPose = RobotHardware.AutonomousDrive.getPoseEstimate();
    }
}


