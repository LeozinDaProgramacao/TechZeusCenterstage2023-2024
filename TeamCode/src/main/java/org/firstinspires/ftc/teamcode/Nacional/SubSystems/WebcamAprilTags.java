package org.firstinspires.ftc.teamcode.Nacional.SubSystems;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class WebcamAprilTags {
    private static AprilTagProcessor aprilTag;
    static double CAM_DIST_TO_CENTER = 6.5;
    static double actualHead;
    static double xtrans;
    static double ytrans;
    private static VisionPortal visionPortal;
    public static void initAprilTag(HardwareMap hardwareMap) {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag
        );

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    @SuppressLint("DefaultLocale")
    public static Pose2d LocateWithAprilTag(Telemetry telemetry, SampleMecanumDriveCancelable autodrive) {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        if (currentDetections.size()>0) {
            double avgX = 0;
            double avgY = 0;
            double avgHead = 0;
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    double xtag = 0;
                    double ytag = 0;
                    switch (detection.id) {
                        case 1:
                            xtag = 63;
                            ytag = -41.5;
                            break;
                        case 2:
                            xtag = 63;
                            ytag = -35.5;
                            break;
                        case 3:
                            xtag = 63;
                            ytag = -29.5;
                            break;
                        case 4:
                            xtag = 63;
                            ytag= -29.5;
                            break;
                        case 5:
                            xtag = 63;
                            ytag =-35.5;
                            break;
                        case 6:
                            xtag = 63;
                            ytag = -41.5;
                            break;
                    }
                    actualHead = -(detection.ftcPose.yaw - detection.ftcPose.bearing);
                    ytrans = Math.sin(Math.toRadians(actualHead)) * detection.ftcPose.range;
                    xtrans = Math.cos(Math.toRadians(actualHead)) * detection.ftcPose.range;

                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                    //telemetry.addLine(String.format("Xtrans %6.1f Ytrans %6.1f",xtrans,ytrans));
                    telemetry.addLine(String.format("CAM FIELD XY %6.1f %6.1f", xtag - xtrans, ytag - ytrans, detection.ftcPose.bearing));
                    telemetry.addLine(String.format("BOT FIELD XY HEAD %6.1f %6.1f %6.1f",
                            (xtag - xtrans) - (Math.cos(Math.toRadians(-detection.ftcPose.yaw)) * CAM_DIST_TO_CENTER),
                            (ytag - ytrans) - (Math.sin(Math.toRadians(-detection.ftcPose.yaw)) * CAM_DIST_TO_CENTER),
                            detection.ftcPose.yaw));
                    //telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    avgX+= (xtag - xtrans) - (Math.cos(Math.toRadians(-detection.ftcPose.yaw)) * CAM_DIST_TO_CENTER);
                    avgY += (ytag - ytrans) - (Math.sin(Math.toRadians(-detection.ftcPose.yaw)) * CAM_DIST_TO_CENTER);
                    avgHead += detection.ftcPose.yaw+180;
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }   // end for() loop

            // Add "key" information to telemetry
            //telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            //telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            //telemetry.addLine("RBE = Range, Bearing & Elevation");

            return new Pose2d(avgX/currentDetections.size(), avgY/currentDetections.size(), -Math.toRadians(avgHead/currentDetections.size()));
        } else {

            return autodrive.getPoseEstimate();
        }
    }   // end method telemetryAprilTag()
}
