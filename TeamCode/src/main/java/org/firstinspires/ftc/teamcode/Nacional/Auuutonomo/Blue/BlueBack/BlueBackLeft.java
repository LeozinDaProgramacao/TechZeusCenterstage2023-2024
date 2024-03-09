package org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Blue.BlueBack;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Nacional.SubSystems.ArmMovement;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BlueBackLeft{

    public static TrajectorySequence getLeftSequence(SampleMecanumDrive drive, Pose2d startPose){

        TrajectorySequence leftSequence= drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(5.5)
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL5UP);
                })
                .lineToLinearHeading(new Pose2d(-42,29,0))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                .setReversed(true)
                .back(3)
                .splineToLinearHeading(new Pose2d(-50,14,Math.toRadians(180)),Math.toRadians(180))
                .forward(4)

                //moves to get the  extra pixel
                .UNSTABLE_addDisplacementMarkerOffset(0,()-> {
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.CLOSED);
                })
                .back(3)
                //.lineToLinearHeading(new Pose2d(-54.5,25.5-12,Math.toRadians(180)))
                //.waitSeconds(1)
                .splineToLinearHeading(new Pose2d(20,6,Math.toRadians(180)),Math.toRadians(0))
                .UNSTABLE_addDisplacementMarkerOffset(-10,()->
                {
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_BACK_AUTO);
                })
                //.splineToConstantHeading(new Vector2d(45,36),0)

                //moves to backdrop to deposit pixel
                .splineToConstantHeading(new Vector2d(47.5,37.5),0)
                .waitSeconds(0.5)
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.OPEN);
                    ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(40,24),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(58,10),Math.toRadians(20))
                .build();

        return leftSequence;
    }
}
