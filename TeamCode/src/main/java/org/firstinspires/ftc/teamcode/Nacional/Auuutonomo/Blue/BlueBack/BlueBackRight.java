package org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Blue.BlueBack;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Nacional.SubSystems.ArmMovement;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class BlueBackRight {
    public static TrajectorySequence getRightSequence(SampleMecanumDrive drive, Pose2d startPose){

        TrajectorySequence rightSequence= drive.trajectorySequenceBuilder(startPose)



                .setReversed(true)
                //.waitSeconds(10)//todo remove me
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL5UP);
                    ArmMovement.setClawMode(ArmMovement.CLAW_MODE.MANUAL);
                })
                .splineToLinearHeading(new Pose2d(-45.2,14,Math.toRadians(90)),Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                .splineToLinearHeading(new Pose2d(-47,14.4,Math.toRadians(180)),Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.setClawMode(ArmMovement.CLAW_MODE.SENSOR);
                })
                .splineToLinearHeading(new Pose2d(-52.2 , 14.4,Math.toRadians(180)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.CORRECT1UP);
                })
                .splineToConstantHeading(new Vector2d(-50,14.4),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL5UP);
                })
                .splineToConstantHeading(new Vector2d(-30,10),Math.toRadians(0),
                        SampleMecanumDrive. getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                //.back(3)
                //.lineToLinearHeading(new Pose2d(-54.5,25.5-12,Math.toRadians(180)))
                //.waitSeconds(1)
                .splineToConstantHeading(new Vector2d(20,6),Math.toRadians(0),
                        SampleMecanumDrive. getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .UNSTABLE_addDisplacementMarkerOffset(-17,()->
                {
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_BACK_AUTO);
                    ArmMovement.setClawMode(ArmMovement.CLAW_MODE.MANUAL);
                })
                //.splineToConstantHeading(new Vector2d(45,36),0)

                //moves to backdrop to deposit pixel
                .splineToConstantHeading(new Vector2d(47.5,27.5),0)
                .splineToConstantHeading(new Vector2d(47.5+6,27.5),0)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.OPEN);
                    ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                //.waitSeconds(0.1)
                .UNSTABLE_addDisplacementMarkerOffset(2,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                })
                //.forward(6)//todo remove this to revert to 2+3
                //todo and this too

                .splineToConstantHeading(new Vector2d(20,6),Math.toRadians(180),
                        SampleMecanumDrive. getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .UNSTABLE_addDisplacementMarkerOffset(16,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL3UP);
                })
                .splineToConstantHeading(new Vector2d(-45,11.2),Math.toRadians(180),
                        SampleMecanumDrive. getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .UNSTABLE_addDisplacementMarkerOffset(0,()-> {
                    ArmMovement.setClawMode(ArmMovement.CLAW_MODE.SENSOR);
                })
                .splineToConstantHeading(new Vector2d(-54.3,11.2),Math.toRadians(180),
                        SampleMecanumDrive. getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .UNSTABLE_addDisplacementMarkerOffset(10,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                    ArmMovement.setClawMode(ArmMovement.CLAW_MODE.MANUAL);
                })

                .lineToSplineHeading(new Pose2d(12,11.2,Math.toRadians(-0.0001)),
                        SampleMecanumDrive.getVelocityConstraint(45, 3, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))

                //.splineToConstantHeading(new Vector2d(40,24),Math.toRadians(-90))
                .UNSTABLE_addDisplacementMarkerOffset(-10,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_FRONT);
                })

                .splineToLinearHeading(new Pose2d(40,38,Math.toRadians(0)),Math.toRadians(0),
                        SampleMecanumDrive. getVelocityConstraint(40, 3, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(55,38, Math.toRadians(0)),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.OPEN);
                    ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                .back(7,
                        SampleMecanumDrive. getVelocityConstraint(3.5, DriveConstants.MAX_ACCEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                })
                .back(0.1)


                /*
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
                .splineToConstantHeading(new Vector2d(49,26.5),0)
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

                 /**/
                .build();

        return rightSequence;
    }
}