package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
@Autonomous(name="rrdwtest")
public class rrdwtest extends LinearOpMode {

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {


        Pose2d startPose = new Pose2d(-35, 64, Math.toRadians(270));

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        /*
        //This trajectory sequence moves the robot to the middle of the square adjacent to all three lines

         */
        double N =30;
        TrajectorySequence startTrajectory = drive.trajectorySequenceBuilder(startPose)
                .forward(N)
                .back(N)
                .forward(N)
                .back(N)
                .forward(N)
                .back(N)
                .forward(N)
                .back(N)
                .build();

        //this trajectory sequence is executed when the game element is detected in the left line




        waitForStart();


        //a câmera detecta em qual posição está o objeto de jogo
        drive.followTrajectorySequence(startTrajectory);

        //alterna entre programações com base na detecção da câmera
    }
}
