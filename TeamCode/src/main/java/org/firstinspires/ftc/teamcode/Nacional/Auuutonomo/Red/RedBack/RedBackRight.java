package org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Red.RedBack;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Nacional.SubSystems.ArmMovement;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RedBackRight {
    public static TrajectorySequence getRightSequence(SampleMecanumDrive drive, Pose2d startPose){
        TrajectorySequence rightSequence= drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL5UP);
                })
                .lineToLinearHeading(new Pose2d(-41,-29,0))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                .setReversed(true)

                .back(3)
                .splineToLinearHeading(new Pose2d(-49,-15,Math.toRadians(180)),Math.toRadians(180))
                .forward(5)

                //moves to get the  extra pixel
                .UNSTABLE_addDisplacementMarkerOffset(0,()-> {
                    ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.CLOSED);
                })
                .back(3)
                //.splineToConstantHeading(new Vector2d(-48,-10),0)
                //.lineToLinearHeading(new Pose2d(-54.5,25.5-12,Math.toRadians(180)))
                //.waitSeconds(1)
                .splineToLinearHeading(new Pose2d(20,-8,Math.toRadians(180)),Math.toRadians(0))
                .UNSTABLE_addDisplacementMarkerOffset(-10,()->
                {
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_BACK);
                })
                //.splineToConstantHeading(new Vector2d(45,36),0)

                //moves to backdrop to deposit pixel
                .splineToConstantHeading(new Vector2d(47,-42),0)
                .waitSeconds(0.5)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.OPEN);
                    ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                })
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(40,-24),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(58,-12),Math.toRadians(-20))
                .build();
        return rightSequence;
    }
}
