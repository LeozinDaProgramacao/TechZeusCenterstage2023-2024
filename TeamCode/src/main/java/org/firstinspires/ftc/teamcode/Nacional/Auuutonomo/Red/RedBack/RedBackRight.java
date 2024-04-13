package org.firstinspires.ftc.teamcode.Nacional.Auuutonomo.Red.RedBack;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Nacional.SubSystems.ArmMovement;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RedBackRight {
    public static TrajectorySequence getRightSequence(SampleMecanumDrive drive, Pose2d startPose){
        TrajectorySequence rightSequence= drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(2,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL5UP);
                })
                .splineToLinearHeading(new Pose2d(-41,-29,0),Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                .turn(Math.toRadians(-180+0.001))
                .splineToLinearHeading(new Pose2d(-49,-13.7,Math.toRadians(180)),Math.toRadians(180))

                .UNSTABLE_addDisplacementMarkerOffset(0,()-> {
                    ArmMovement.setClawMode(ArmMovement.CLAW_MODE.SENSOR);
                })
                .splineToLinearHeading(new Pose2d(-53,-13.7,Math.toRadians(180)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                //moves to get the  extra pixel
                .splineToConstantHeading(new Vector2d(-50,-13.7),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .splineToConstantHeading(new Vector2d(-30,-10),Math.toRadians(0),
                        SampleMecanumDrive. getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                //.back(3)
                //.lineToLinearHeading(new Pose2d(-54.5,25.5-12,Math.toRadians(180)))
                //.waitSeconds(1)
                .splineToConstantHeading(new Vector2d(20,-6),Math.toRadians(0),
                        SampleMecanumDrive. getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))
                .UNSTABLE_addDisplacementMarkerOffset(-17,()->
                {
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_BACK_AUTO);
                    ArmMovement.setClawMode(ArmMovement.CLAW_MODE.MANUAL);
                })
                //.splineToConstantHeading(new Vector2d(45,36),0)

                //moves to backdrop to deposit pixel
                .splineToConstantHeading(new Vector2d(47.5,-41),0)
                .splineToConstantHeading(new Vector2d(47.5+6,-41),0)
                .setReversed(false)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    ArmMovement.ControlLeftClaw(ArmMovement.CLAW_STATE.OPEN);
                    ArmMovement.ControlRightClaw(ArmMovement.CLAW_STATE.OPEN);
                })
                //.waitSeconds(0.1)
                .UNSTABLE_addDisplacementMarkerOffset(2,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                })



                .splineToConstantHeading(new Vector2d(20,-6),Math.toRadians(180),
                        SampleMecanumDrive. getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .UNSTABLE_addDisplacementMarkerOffset(16,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.PIXEL3UP);
                })
                .splineToConstantHeading(new Vector2d(-45,-8.3),Math.toRadians(180),
                        SampleMecanumDrive. getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(35))
                .UNSTABLE_addDisplacementMarkerOffset(0,()-> {
                    ArmMovement.setClawMode(ArmMovement.CLAW_MODE.SENSOR);
                })
                .splineToConstantHeading(new Vector2d(-53.8,-8.3),Math.toRadians(180),
                        SampleMecanumDrive. getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .UNSTABLE_addDisplacementMarkerOffset(10,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.STORED);
                    ArmMovement.setClawMode(ArmMovement.CLAW_MODE.MANUAL);
                })

                .lineToSplineHeading(new Pose2d(12,-9,Math.toRadians(0.0001)),
                        SampleMecanumDrive.getVelocityConstraint(45, 3, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(30))

                //.splineToConstantHeading(new Vector2d(40,24),Math.toRadians(-90))
                .UNSTABLE_addDisplacementMarkerOffset(-10,()->{
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_FRONT);
                })

                .splineToLinearHeading(new Pose2d(40,-27,Math.toRadians(0)),Math.toRadians(0),
                        SampleMecanumDrive. getVelocityConstraint(40, 3, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(53,-27, Math.toRadians(0)),Math.toRadians(0))
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
                .back(3)
                //.splineToConstantHeading(new Vector2d(-48,-10),0)
                //.lineToLinearHeading(new Pose2d(-54.5,25.5-12,Math.toRadians(180)))
                //.waitSeconds(1)
                .splineToLinearHeading(new Pose2d(20,-8,Math.toRadians(180)),Math.toRadians(0))
                .UNSTABLE_addDisplacementMarkerOffset(-10,()->
                {
                    ArmMovement.setArmState(ArmMovement.ARM_STATE.DEPOSIT_BACK_AUTO);
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

                 */
                .build();
        return rightSequence;
    }
}
