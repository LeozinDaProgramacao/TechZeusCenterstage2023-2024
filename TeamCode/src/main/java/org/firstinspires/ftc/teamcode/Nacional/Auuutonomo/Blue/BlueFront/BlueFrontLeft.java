package org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Blue.BlueFront;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Nacional.SubSystems.ArmMovement;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BlueFrontLeft {
    public static TrajectorySequence getLeftSequence(SampleMecanumDrive drive, Pose2d startPose){
        TrajectorySequence leftSequencee= drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL4UP);
                })
                .lineToLinearHeading(new Pose2d(38,32,Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_BACK_AUTO);
                })
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(49,41),0)
                .waitSeconds(0.1)
                .back(6)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                })
                .splineToConstantHeading(new Vector2d(13,60),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30,60),Math.toRadians(180))
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL4UP);
                })
                .splineToLinearHeading(new Pose2d(-53,36,Math.toRadians(180)),Math.toRadians(180))


                .waitSeconds(0.1)
                .forward(1.2)
                //.setReversed(true)
                //.back(3)
                //.splineToLinearHeading(new Pose2d(-56,12,Math.toRadians(180)),Math.toRadians(180))

                //moves to get the  extra pixel

                .UNSTABLE_addTemporalMarkerOffset(0,()-> {
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.CLOSED);
                })
                .waitSeconds(0.1)
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                })
                .back(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-30,60,Math.toRadians(180)),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(13,60),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_BACK_AUTO);
                })
                .splineToConstantHeading(new Vector2d(47,32.5),0)
                .back(4)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.OPEN);
                }).waitSeconds(0.1)
                .forward(5)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                })
                .splineToConstantHeading(new Vector2d(40,60),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60,60),Math.toRadians(0))
                .build();
        return leftSequencee;
    }
}
