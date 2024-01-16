package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.mycompany.node.Node;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp

public class GraphRunner extends LinearOpMode {
    SampleMecanumDrive drive;



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(72, 72, Math.toRadians(270));
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        waitForStart();
        Node end = Node.makebluegraph();

        Trajectory ass = Node.printPath(
                Node.aStar(
                        Node.getClosestNode(
                                new Node(72,72)
                        ),end)
                ,drive, new Pose2d(72,72,0));


    }
}
