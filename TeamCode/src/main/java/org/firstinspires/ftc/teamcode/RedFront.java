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
@Autonomous(name="RedFront", group="RedAuto")
public class RedFront extends LinearOpMode {
    OpenCvWebcam webcam;
    private  Servo Articulation;
    private Servo LClaw;
    private Servo RClaw;

    RedDetector.Location registred;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        Articulation = hardwareMap.get(Servo.class, "Articulation");
        LClaw = hardwareMap.get(Servo.class, "LClaw");
        RClaw = hardwareMap.get(Servo.class, "RClaw");


        Pose2d startPose = new Pose2d(-35, -64, Math.toRadians(270));

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        RedDetector detector = new RedDetector(telemetry);
        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
        /*
        //This trajectory sequence moves the robot to the middle of the square adjacent to all three lines

         */
        Trajectory startTrajectory = drive.trajectoryBuilder(startPose)
                .forward(29)
                .build();

        //this trajectory sequence is executed when the game element is detected in the left line
        TrajectorySequence lftSequence = drive.trajectorySequenceBuilder(startTrajectory.end())
                .lineToLinearHeading(startPose.plus(new Pose2d(0,36,Math.toRadians(90))))
                .forward(2)
                .addDisplacementMarker(()->{
                            openClaw();
                        }
                )//ta joia
                .waitSeconds(0.5)
                .back(2)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1,()->
                {
                    closeClaw();
                })
                .back(1.5)
                .addDisplacementMarker(()->{
                            ArticulationUp();
                        }
                )
                .lineToLinearHeading(startPose.plus(new Pose2d(+1.5,36,Math.toRadians(90))).plus(new Pose2d(42,0,Math.toRadians(180))))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                            ArticulationDown();
                        }
                )
                .UNSTABLE_addTemporalMarkerOffset(-1+0.5,()->{
                    openClaw();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1+0.5,()->
                        {
                            ArticulationStore();
                        }
                ).UNSTABLE_addTemporalMarkerOffset(0,()->{
                            closeClaw();
                        }
                ).back(5)
                .strafeRight(30)
                .forward(10)
                .build();


        //MID SEAQUENCE
        TrajectorySequence midSequence = drive.trajectorySequenceBuilder(startTrajectory.end())
                .forward(27)
                .forward(2)
                .addDisplacementMarker(()->{
                            openClaw();
                        }
                )//ta joia
                .waitSeconds(0.5)
                .back(2.5)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(
                        -1,()->{
                            closeClaw();
                        }
                ).back(1)
                .addDisplacementMarker(()->{
                            ArticulationUp();
                        }
                ).lineToLinearHeading(startPose.plus(new Pose2d(0,27.5,0)).plus(new Pose2d(36,0,Math.toRadians(-90))))
                .forward(10)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                            ArticulationDown();
                        }
                )
                .UNSTABLE_addTemporalMarkerOffset(-1+0.5,()->{
                    openClaw();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1+0.5,()->
                        {
                            ArticulationStore();
                        }
                ).UNSTABLE_addTemporalMarkerOffset(0,()->{
                            closeClaw();
                        }
                )
                .back(5)
                .strafeRight(23)
                .forward(10)
                .build();


        TrajectorySequence rgtSequence = drive.trajectorySequenceBuilder(startTrajectory.end())
                .lineToLinearHeading(startPose.plus(new Pose2d(24,32,Math.toRadians(90))))
                .forward(2.5)
                .addDisplacementMarker(()->{
                            openClaw();
                        }
                )//ta joia
                .waitSeconds(0.5)
                .back(2)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-1,()->{
                    closeClaw();
                })
                .back(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5,()->{
                    ArticulationUp();
                })
                .lineToLinearHeading(startPose.plus(new Pose2d(24,32,Math.toRadians(90)).plus(new Pose2d(11,-12,Math.toRadians(180)))))
                .forward(10)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->{
                    ArticulationDown();
                })
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    openClaw();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(-0.5,()->
                        {
                            ArticulationStore();
                        }
                ).UNSTABLE_addTemporalMarkerOffset(0,()->
                {
                    closeClaw();
                }).back(5)
                .strafeRight(16)
                .forward(15)
                .build();




        closeClaw();

        waitForStart();

        ArticulationDown();
        sleep(5000);

        //a câmera detecta em qual posição está o objeto de jogo

        //alterna entre programações com base na detecção da câmera
        switch (registred) {
            case LEFT:
                //posição esquerda
                drive.followTrajectorySequence(lftSequence);
                break;
            case MIDDLE:
                //posição central
                drive.followTrajectorySequence(midSequence);
                break;
            case RIGHT:
                //posição direita
                drive.followTrajectorySequence(rgtSequence);
                break;
        }
    }




    public void openClaw(){
        LClaw.setPosition(0.5);
        RClaw.setPosition(0.5);
    };
    public void closeClaw(){
        LClaw.setPosition(0.65);
        RClaw.setPosition(0.35);
    }
    public void ArticulationDown(){
        Articulation.setPosition(0.75);
    }
    public void ArticulationGrab(){ Articulation.setPosition(0.83); }
    public void ArticulationMid(){
        Articulation.setPosition(0.6);
    }
    public void ArticulationUp(){
        Articulation.setPosition(0.43);
    }
    public void ArticulationStore(){
        Articulation.setPosition(0.2);
    }

}
