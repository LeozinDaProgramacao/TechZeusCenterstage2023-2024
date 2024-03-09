package org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Blue.BlueFront;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Nacional.SubSystems.ArmMovement;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BlueFrontRight {
    public static TrajectorySequence getRightSequence(SampleMecanumDrive drive, Pose2d startPose){
        TrajectorySequence rightSequence= drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL4UP);
                })
                .splineToLinearHeading(new Pose2d(15.5,31,Math.toRadians(180)),Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                .waitSeconds(0.12)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_BACK_AUTO);
                })
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(49,28,Math.toRadians(180)),0)
                .waitSeconds(0.15)
                .back(6)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.OPEN);
                })









                .waitSeconds(0.15)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL4UP);
                })
                .splineToConstantHeading(new Vector2d(13,60),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30,60),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-53,36.7,Math.toRadians(180)),Math.toRadians(180))

                .waitSeconds(0.1)
                .forward(1.2)
                //.setReversed(true)
                //.back(3)
                //.splineToLinearHeading(new Pose2d(-56,12,Math.toRadians(180)),Math.toRadians(180))

                //moves to get the  extra pixel

                .UNSTABLE_addTemporalMarkerOffset(0,()-> {
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.CLOSED);
                })
                .waitSeconds(0.15)
                .setReversed(true)
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                })
                .splineToLinearHeading(new Pose2d(-30,60,Math.toRadians(180)),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(13,60),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_BACK_AUTO);
                })
                .splineToConstantHeading(new Vector2d(47,39),0)
                .waitSeconds(0.2)
                .back(8)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.OPEN);
                }).waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                })
                .forward(10)
                .splineToConstantHeading(new Vector2d(40,60),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60,60),Math.toRadians(0))
                .build();

        return rightSequence;
    }
}
