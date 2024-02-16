package org.firstinspires.ftc.teamcode.oldcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous (name="DetectConfig")
public class DetectConfig extends LinearOpMode {
    OpenCvWebcam webcam;
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        RedDetector detector = new RedDetector(telemetry);
        //RedDetector detector = new RedDetector(telemetry);

        webcam.setPipeline(detector);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });


        waitForStart();



        int gain = webcam.getGainControl().getGain();
        //long exposure = webcam.getExposureControl().getExposure();
        telemetry.addData("gain",gain);
        telemetry.update();

        if (isStopRequested()) return;

        while(opModeIsActive()){
            FtcDashboard.getInstance().startCameraStream(webcam,0);
            sleep(100);
        }
    }
}
