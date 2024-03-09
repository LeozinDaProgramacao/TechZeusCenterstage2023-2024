package org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Blue.BlueBack;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Nacional.SubSystems.ArmMovement;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BlueBackMid {
    public static TrajectorySequence getMidSequence(SampleMecanumDrive drive, Pose2d startPose){

        TrajectorySequence mideSequence= drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(5.5)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL5UP);
                })
                .lineToLinearHeading(new Pose2d(-39,8,Math.toRadians(90)))

                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                .waitSeconds(0.2)
                .turn(Math.toRadians(90))
                //moves to get the  extra pixel
                .splineToLinearHeading(new Pose2d(-51,15,Math.toRadians(180)),Math.toRadians(180))
                .forward(3)
                .UNSTABLE_addDisplacementMarkerOffset(0,()-> {
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.CLOSED);
                })
                .setReversed(true)
                //.lineToLinearHeading(new Pose2d(-54.5,25.5-12,Math.toRadians(180)))
                //.waitSeconds(1)
                .splineToLinearHeading(new Pose2d(20,6,Math.toRadians(180)),Math.toRadians(0))
                .UNSTABLE_addDisplacementMarkerOffset(-10,()->
                {
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_BACK_AUTO);
                })
                //.splineToConstantHeading(new Vector2d(45,36),0)
                .splineToConstantHeading(new Vector2d(49,31),0)
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
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(40,24),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(58,10),Math.toRadians(20))
                .build();

        return mideSequence;
    }
}