/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Config
@Disabled
@TeleOp
public class WebcamExample extends LinearOpMode
{
    OpenCvWebcam webcam;
    public static double LTLX=0;
    public static double LTLY=0;
    public static double LBRX=200;
    public static double LBRY=200;
    double[] fartt = new double[3];
    int lobby;
    int mobby;
    int robby;


    @Override
    public void runOpMode()
    {
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        SamplePipeline sex = new SamplePipeline();
        webcam.setPipeline(sex);
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {}
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            /*telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("0",fartt[0]);
            telemetry.addData("0",fartt[1]);
            telemetry.addData("0",fartt[2]);
            telemetry.addData("bobby",bobby);
            telemetry.update();
            bobby=0;*/


            /*
             * NOTE: stopping the stream from the camera early (before the end of the OpMode
             * when it will be automatically stopped for you) *IS* supported. The "if" statement
             * below will stop streaming from the camera when the "A" button on gamepad 1 is pressed.
             */
            if(gamepad1.a)
            {
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }



            /*
             * For the purposes of this sample, throttle ourselves to 10Hz loop to avoid burning
             * excess CPU cycles for no reason. (By default, telemetry is only sent to the DS at 4Hz
             * anyway). Of course in a real OpMode you will likely not want to do this.
             */
            FtcDashboard.getInstance().startCameraStream(webcam,0);
            sleep(100);
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {

        boolean viewportPaused;

        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        @Override
        public Mat processFrame(Mat input)
        {

            Scalar lowRGB = new Scalar(200,60,60);
            Scalar highRGB = new Scalar(255,100,100);

            Mat mat = new Mat();
            input.copyTo(mat);
            Mat left = mat.submat(200,600,0,426);
            double num = Core.sumElems(input).val[0];
            num =Core.countNonZero(input);
            Scalar green = new Scalar(0,255,0);
/*
            for (int X=0;X<=426;X++){
                for(int Y =200;Y<=600;Y++){
                    double[] Pointvalues = input.get(X,Y);
                    if ((Pointvalues[0]>=190&&Pointvalues[0]<=255)&&
                            (Pointvalues[1]>=50&&Pointvalues[1]<=110)&&
                            (Pointvalues[2]>=50&Pointvalues[2]<=110)){
                        lobby++;
                    }
                  Y+=10;
                }
                X+=10;
            }
            //all good up till here

            /**//*
            for (int XX=427;XX<=852;XX++){
                for(int YY =200;YY<=600;YY++){
                    double[] Pointvalues = input.get(XX,YY);
                    if ((Pointvalues[0]>=190&&Pointvalues[0]<=255)&&
                            (Pointvalues[1]>=50&&Pointvalues[1]<=110)&&
                            (Pointvalues[2]>=50&Pointvalues[2]<=110)){
                        mobby++;
                    }
                  YY+=4;
                }
                XX+=4;
            }/**//*
            for (int X=427;X<=852;X++){
                for(int Y =200;Y<=600;Y++){
                    double[] Pointvalues = input.get(X,Y);
                    if ((Pointvalues[0]>=190&&Pointvalues[0]<=255)&&
                            (Pointvalues[1]>=50&&Pointvalues[1]<=110)&&
                            (Pointvalues[2]>=50&Pointvalues[2]<=110)){
                        mobby++;
                    }
                    Y+=10;
                }
                X+=10;
            }/**//*
            for (int XXX=853;XXX<=1279;XXX++){
                for(int YYY =200;YYY<=600;YYY++){
                    double[] Pointvalues = input.get(XXX,YYY);
                    if ((Pointvalues[0]>=190&&Pointvalues[0]<=255)&&
                            (Pointvalues[1]>=50&&Pointvalues[1]<=110)&&
                            (Pointvalues[2]>=50&Pointvalues[2]<=110)){
                        robby++;
                    }
                  //  Y+=4;
                }
                //X+=4-;
            }/**/
            //left rectangle
            if (lobby>mobby){
                if (lobby>robby){
                    //left rectangle
                    Imgproc.rectangle(input,new Point(000,200),new Point(426,600),green,5,Imgproc.LINE_8);

                }
            } else if (mobby>robby){
                //middle rectangle
                Imgproc.rectangle(input,new Point(427,200),new Point(852,600),green,5,Imgproc.LINE_8);
            } else {
                //right rectangle
                Imgproc.rectangle(input,new Point(853,200),new Point(1280,600),green,5,Imgproc.LINE_8);

            }/**/
            //Imgproc.rectangle(input,new Point(000,200),new Point(426,600),green,5,Imgproc.LINE_8);
            //middle rectangle
            //Imgproc.rectangle(input,new Point(427,200),new Point(852,600),green,5,Imgproc.LINE_8);
            //right rectangle
            //Imgproc.rectangle(input,new Point(853,200),new Point(1280,600),green,5,Imgproc.LINE_8);

            Imgproc.circle(input,new Point(640,360),10,green,5,Imgproc.LINE_8);
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.addData("0",fartt[0]);
            telemetry.addData("0",fartt[1]);
            telemetry.addData("0",fartt[2]);
            telemetry.addData("num",num);
            telemetry.addData("bobby",lobby);
            telemetry.addData("mobby",mobby);
            telemetry.addData("robby",robby);
            telemetry.update();
            fartt = input.get(360,640);
            lobby=0;
            mobby=0;
            robby=0;
            return input;
        }

        @Override
        public void onViewportTapped()
        {
            /*
             * The viewport (if one was specified in the constructor) can also be dynamically "paused"
             * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
             * when you need your vision pipeline running, but do not require a live preview on the
             * robot controller screen. For instance, this could be useful if you wish to see the live
             * camera preview as you are initializing your robot, but you no longer require the live
             * preview after you have finished your initialization process; pausing the viewport does
             * not stop running your pipeline.
             *
             * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
             */

            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }
    }
}
