package org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Red.RedFront;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Nacional.SubSystems.ArmMovement;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RedFrontLeft{
    public static TrajectorySequence getLeftSequence(SampleMecanumDrive drive, Pose2d startPose){

        TrajectorySequence leftSequence= drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL4UP);
                    ArmMovement.setClawMode(ArmMovement.CLAW_MODE.MANUAL);
                })
                .splineToLinearHeading(new Pose2d(15.5,-29,Math.toRadians(180)),Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_BACK);
                })
                .waitSeconds(0.3)
                .splineToLinearHeading(new Pose2d(49,-26,Math.toRadians(180)),0)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                .UNSTABLE_addDisplacementMarkerOffset(2,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                })
                //////////////////
                .splineToConstantHeading(new Vector2d(13,-58),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-30,-58),Math.toRadians(180))


                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL4UP);

                })
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    ArmMovement.setClawMode(ArmMovement.CLAW_MODE.SENSOR);
                })
                .splineToLinearHeading(new Pose2d(-47.5,-36.5,Math.toRadians(180)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-54,-36.5,Math.toRadians(180)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                //.setReversed(true)
                //.back(3)
                //.splineToLinearHeading(new Pose2d(-56,12,Math.toRadians(180)),Math.toRadians(180))

                //moves to get the  extra pixel
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-47,-36.7,Math.toRadians(180)),0)
                //.setReversed(true)
                //.back(3)
                //.splineToLinearHeading(new Pose2d(-56,12,Math.toRadians(180)),Math.toRadians(180))

                //moves to get the  extra pixel


                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.setClawMode(ArmMovement.CLAW_MODE.MANUAL);
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                })
                .splineToLinearHeading(new Pose2d(-35,-58,Math.toRadians(180)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .splineToLinearHeading(new Pose2d(15,-58,Math.toRadians(180)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))


                // undo this comment to make it regular
                /*
                .UNSTABLE_addTemporalMarkerOffset(1.7,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_FRONT);
                })
                .splineToLinearHeading(new Pose2d(42,-35,0.0001),0,
                        SampleMecanumDrive. getVelocityConstraint(40, 3, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))

                .splineToLinearHeading(new Pose2d(56,-35, Math.toRadians(0)),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.OPEN);
                    ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                .back(8,
                        SampleMecanumDrive. getVelocityConstraint(3, DriveConstants.MAX_ACCEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                })
                .splineToConstantHeading(new Vector2d(45,-40),Math.toRadians(-90),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                .splineToConstantHeading(new Vector2d(55,-60),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(40))
                /**/


                //this is for the WX coop auto
                .splineToLinearHeading(new Pose2d(43,-59,Math.toRadians(0)),Math.toRadians(0))
                .waitSeconds(1.1)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL1UP);
                })
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.OPEN);
                    ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                .waitSeconds(1.2)





                .build();


















                /*
                .UNSTABLE_addTemporalMarkerOffset(0,()-> {
                    ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.CLOSED);
                })
                .waitSeconds(0.15)

                .setReversed(true)
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                })
                .splineToLinearHeading(new Pose2d(-30,-60,Math.toRadians(180)),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(13,-60),Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_BACK);
                })
                .splineToConstantHeading(new Vector2d(47,-39),0)
                .waitSeconds(0.3)
                .back(8)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.OPEN);
                }).waitSeconds(0.2)
                .UNSTABLE_addDisplacementMarkerOffset(0,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                })
                .forward(10)
                .splineToConstantHeading(new Vector2d(40,-60),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(60,-60),Math.toRadians(0))
                .build();

                 */

        return leftSequence;
    }
}