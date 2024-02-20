package org.firstinspires.ftc.teamcode.oldcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class RedDetector extends OpenCvPipeline {
    public static int A=0;
    public static int B=95;
    public static int C=60;
    public static int D=15;
    public static int E=255;
    public static int F=255;

    Telemetry telemetry;
    Scalar green = new Scalar(0,255,0);
    Mat mat = new Mat();
    public enum Location{
        LEFT,MIDDLE,RIGHT
    }
    private RedDetector.Location location;
    final Rect LEFT = new Rect(
            new Point(000,200),
            new Point(426,600));
    final Rect MIDDLE = new Rect(
            new Point(427,200),
            new Point(852,600));
    final Rect RIGHT = new Rect(
            new Point(853,200),
            new Point(1280,600));
    double PERCENT_COLOR_THRESH =0.01;
    public RedDetector(Telemetry t){telemetry=t;}

    @Override
    public  Mat processFrame(Mat input){


        Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV = new Scalar(A,B,C);
        Scalar highHSV = new Scalar(D,E,F);
        Core.inRange(mat,lowHSV,highHSV,mat);
        Mat mask = new Mat();
        mat.copyTo(mask);
        Mat left = mat.submat(LEFT);
        Mat middle = mat.submat(MIDDLE);
        Mat right = mat.submat(RIGHT);

        double leftValue = Core.sumElems(left).val[0]/LEFT.area()/255;
        double middleValue = Core.sumElems(middle).val[0]/MIDDLE.area()/255;
        double rightValue = (Core.sumElems(right).val[0]-5000)/RIGHT.area()/255;

        telemetry.addData("lftraw",(int) Core.sumElems(left).val[0]);
        telemetry.addData("midraw",(int) Core.sumElems(middle).val[0]);
        telemetry.addData("rgtraw",(int) Core.sumElems(right).val[0]);
        telemetry.addData("lftval",leftValue);
        telemetry.addData("midval",middleValue);
        telemetry.addData("rgtval",rightValue);
        telemetry.addData("lftPercent",Math.round(leftValue*100)+"%");
        telemetry.addData("midPercent",Math.round(middleValue*100)+"%");
        telemetry.addData("rgtPercent",Math.round(rightValue*100)+"%");
        left.release();
        middle.release();
        right.release();
        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);
        if (leftValue>middleValue){
            if (leftValue>rightValue){
                Imgproc.rectangle(input,LEFT,green);
                location= RedDetector.Location.LEFT;
                telemetry.addData("side","left");
            } else {
                Imgproc.rectangle(input,RIGHT,green);
                location= RedDetector.Location.RIGHT;
                telemetry.addData("side","right");
            }
        } else if (middleValue>rightValue){
            Imgproc.rectangle(input,MIDDLE,green);
            location = RedDetector.Location.MIDDLE;
            telemetry.addData("side","middle");
        } else {
            Imgproc.rectangle(input,RIGHT,green);
            location= RedDetector.Location.RIGHT;
            telemetry.addData("side","right");
        }
        telemetry.update();
        //input.release();
        mask.release();
        mat.release();
        return input;
    }
    public RedDetector.Location getLocation(){
        return location;
    }
}
