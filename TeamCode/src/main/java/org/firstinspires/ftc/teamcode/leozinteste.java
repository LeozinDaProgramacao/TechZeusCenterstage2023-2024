
package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "leozinteste")
@Disabled

public class leozinteste extends LinearOpMode {
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private CRServo LArm;
    private CRServo RArm;
    private Servo servoArticulacao;
    private Servo servoGarra;
    private IMU imu;
    YawPitchRollAngles orientation;
    double drive;
    double turn;
    double strafe;
    double DeltaAngle;
    double CurrentFront;
    double CurrentAngle;
    double fL;
    double fR;
    double bL;
    double bR;
    boolean yOn;
    double powerbraco;
    double finalgoalbraco = 100;
    public static double A=0;
    public static double B=0;
    OpenCvWebcam webcam;
    double fart;
    double goalbraco;
    final double maximoo = 0.5;
    //PID pid_turn = new PID(0.5,0.005,0.0,0.4);
    PID pid_turn = new PID(0.5,0.000,0.0,0.3);
    //PID pid_braco = new PID(0.005,0.01,0.003);
    PID pid_braco = new PID(0.01,0.0006,0,0.4);
    double prevtime=0;
    boolean cam_running=true;
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    /**
     * This function is executed when this Op Mode is selected.
     */
    @Override
    public void runOpMode() {
        //inicializa hardware
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        frontRight = hardwareMap.get(DcMotor.class, "FRmotor");
        backRight = hardwareMap.get(DcMotor.class, "BRmotor");
        frontLeft = hardwareMap.get(DcMotor.class, "FLmotor");
        backLeft = hardwareMap.get(DcMotor.class, "BLmotor");
        //LArm = hardwareMap.get(CRServo.class, "LArm");
        //RArm = hardwareMap.get(CRServo.class, "RArm");
        //servoArticulacao = hardwareMap.get(Servo.class, "servoArticulacao");
        //servoGarra = hardwareMap.get(Servo.class, "servoGarra");


        //corrige orientação dos motores
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        //RArm.setDirection(DcMotorSimple.Direction.REVERSE);
        //LArm.setDirection(DcMotorSimple.Direction.FORWARD);

        //falsifica um encoder pro braço
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //servoArticulacao.setDirection(DcMotorSimple.Direction.REVERSE);
        //configura câmera
        //webcam.setPipeline(new leozinteste.SamplePipeline());
        //webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        /*webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {}
        });*/


        //Ativa movimentação com Encoders (a fazer)
        //
        //
        //

        //define variaveis essenciais e inicializa funções

        YawPitchRollAngles orientation = null;
        AngularVelocity angularVelocity;

        waitForStart();
        //falsifica um encoder pro braço
        double CurrentFront = 0;
        
        while  (opModeIsActive()) {
            //braco();
            //garra();
            //get gamepad axis
            inputs();
            manageIMU();
            //camera();
            moveBase();
            //adicionarTelemetria();
            telemetry.addData("fl",frontLeft.getPower());
            telemetry.addData("fr",frontRight.getPower());
            telemetry.addData("bl",backLeft.getPower());
            telemetry.addData("br",backRight.getPower());
            telemetry.update();
        }
    }

    private void camera() {
        if (!gamepad1.x){
            webcam.stopStreaming();
            cam_running = false;
        }
        if(gamepad1.x)
        {
            if (!cam_running){
                cam_running = true;
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            //webcam.closeCameraDevice();
            if (fart<600){
                turn = -0.5;
            } else if (fart >800){
                turn = 0.5;
            }
        }
        FtcDashboard.getInstance().startCameraStream(webcam,0);
    }

    private void inputs(){
        //get gamepad axis
        drive = gamepad1.left_stick_y*-1;
        turn = gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        orientation = imu.getRobotYawPitchRollAngles();
        DeltaAngle =360- (orientation.getYaw(AngleUnit.RADIANS)*180/3.141);
    }
    private void manageIMU(){

        if(gamepad1.a){
            imu.resetYaw();
            CurrentFront = orientation.getYaw(AngleUnit.RADIANS);
        }

        //ao girar manualmente, reseta a orientação tida como frente pelo robô (para uso no PID)
        if (Math.abs(gamepad1.right_stick_x)>=0.09){
            CurrentFront = orientation.getYaw(AngleUnit.RADIANS);
            pid_turn.cummulativeError=0;
        }
        //Parametros para o PID são calculados (não é tudo feito dentro do PID para permitir uso na telemetria)
        CurrentAngle = orientation.getYaw(AngleUnit.RADIANS);
        if (CurrentAngle <CurrentFront){
            CurrentAngle = 2*Math.PI+CurrentAngle-CurrentFront;
        } else{
            CurrentAngle-=CurrentFront;
        }

        //se não houver uma curva manual, um PID é usado para manter a orientação do robô
        if (Math.abs(turn)<0.09){
            turn = pid_turn.CalculatePID(ConvertAngle(CurrentAngle),0,true)*-1;
        }
    }
    private void moveBase(){
        //muda a direção de movimento com base na orientação ro robo
        double rotX = strafe*Math.cos(orientation.getYaw(AngleUnit.RADIANS))+drive*Math.sin(orientation.getYaw(AngleUnit.RADIANS));
        double rotY = strafe*Math.sin(-orientation.getYaw(AngleUnit.RADIANS))+drive*Math.cos(-orientation.getYaw(AngleUnit.RADIANS));

        //define as velocidades de cada motor
        double maximo = Math.max(Math.abs(rotX)+Math.abs(rotY)+Math.abs(turn),1);
        fL= (rotY+rotX+turn)/maximo;
        fR=(rotY-rotX-turn)/maximo;
        bL=(rotY-rotX+turn)/maximo;
        bR=(rotY+rotX-turn)/maximo;

        //aplica as potências aos motores
        frontRight.setPower(fR*maximoo*1);
        frontLeft.setPower(fL*maximoo*1);
        backRight.setPower(bR*maximoo*1);
        backLeft.setPower(bL*maximoo*1);/**/
    }
    @SuppressLint("DefaultLocale")
    private void adicionarTelemetria(){
        /*Description:
        this funtion adds all debugging data needed to the telemetry board
        */

        //imu

        telemetry.addData("Delta Angle", DeltaAngle); //quanto a base deslocou em radianos desde o ultimo reset
        telemetry.addData("turn PID detected Error", pid_turn.error*180/Math.PI);
        telemetry.addData("current angle",CurrentAngle*180/Math.PI);
        telemetry.addData("currentfront",CurrentFront);
        telemetry.addData("cumError",pid_turn.cummulativeError);
        /* */

        //valores crus (direto do gamepad)
        /*
        telemetry.addData("raw FB",-1*gamepad1.left_stick_y);
        telemetry.addData("raw LR",gamepad1.left_stick_x);
        telemetry.addData("raw TR",turn);
        //telemetry.addData("multiplier",maximo);//garante que a velocidade maxima seja 1/**/



        //velocidades aplicadas nas rodas
        telemetry.addData("delay=",getRuntime()-prevtime);
        prevtime=getRuntime();
        telemetry.addData("fL",fL);
        telemetry.addData("fR",fR);
        telemetry.addData("bL",bL);
        telemetry.addData("bR",bR);
        telemetry.addData("armpos",backRight.getCurrentPosition());
        telemetry.addData("fingoal",finalgoalbraco);
        telemetry.addData("goalbraco",goalbraco);
        telemetry.addData("erro",pid_braco.error);
        telemetry.addData("powerbraco",powerbraco);
        telemetry.addData("LARM power",LArm.getPower());
        telemetry.addData("RARM power",RArm.getPower());

        //telemetria camera
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        telemetry.addData("fart",fart);



        telemetry.update(); // adiciona tudo da telemetria
    }
    private void braco() {
        //alto = 110?
        //baixo = 0
        if (gamepad2.y||(yOn)) {
            yOn=false;
            yOn =  gamepad2.y;
            if (yOn&&gamepad2.y){
                LArm.setPower(-0.2);
                RArm.setPower(0.2);
            } else {
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        } else {
        finalgoalbraco = gamepad2.right_trigger*540;// o numero é o limite
        goalbraco+= 0.037*(finalgoalbraco-goalbraco);//o valor numerico dessa linha aquiiiiii
        /*if (!((goalbraco>finalgoalbraco-1)&&(goalbraco<finalgoalbraco+1))){
            if (goalbraco>finalgoalbraco){
                goalbraco-=1;
            }else{
                goalbraco+=1;
            }
        }*/
        powerbraco = pid_braco.CalculatePID(backRight.getCurrentPosition(),goalbraco,false);

        LArm.setPower(powerbraco);
        RArm.setPower(-powerbraco);
        }
    }
    private void garra(){
        if (gamepad2.a) {
            servoGarra.setPosition(0);
        } else if (gamepad2.b) {
            servoGarra.setPosition(1);
        }
        if (gamepad2.dpad_down) {
            servoArticulacao.setPosition(0.5);
        } else if (gamepad2.dpad_up) {
            servoArticulacao.setPosition(1.2);
        }else if (gamepad2.dpad_left) {
            servoArticulacao.setPosition(0.7);
        }
    }
  public double ConvertAngle(double angle){
    double return_angle = angle;
    if(angle>Math.PI){
        return_angle -= 2*Math.PI;
    }
    return return_angle;
  }


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
            Mat mat = new Mat();

            Imgproc.cvtColor(input,mat,Imgproc.COLOR_RGB2HSV);
            if (mat.empty()){
                return input;
            }
            Scalar lowHSV = new Scalar(15,50,40);
            Scalar HighHSV = new Scalar(40,255,255);

            Mat thresh =new Mat();

            Core.inRange(mat,lowHSV,HighHSV,thresh);
            //Mat masked = new Mat();
            //Core.bitwise_and(mat,mat,masked,thresh);

            Mat edges = new Mat();

            Imgproc.Canny(thresh,edges,A,B);

            List<MatOfPoint> countours = new ArrayList<>();
            Mat hie = new Mat();
            Imgproc.findContours(thresh,countours,hie,Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);
            //thresh.copyTo(input);

            double maxVal = 0;
            int maxValIdx = 0;
            for (int contourIdx = 0; contourIdx < countours.size(); contourIdx++)
            {
                double contourArea = Imgproc.contourArea(countours.get(contourIdx));
                if (maxVal < contourArea)
                {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }
            }
            Moments M = Imgproc.moments(countours.get(maxValIdx));
            Point center = new Point();
            center.x = M.get_m10() / M.get_m00();
            fart = center.x;
            center.y = M.get_m01()/M.get_m00();

            //Imgproc.drawContours(input, countours, maxValIdx, new Scalar(0,255,0), 5);
            Scalar green = new Scalar(0,255,0);
            Imgproc.circle(input,center,1,green,5,Imgproc.LINE_8);

            hie.release();
            thresh.release();
            //masked.release();
            edges.release();
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

class PID{
    double kP;
    double kI;
    double kD;
    double MaxI;
    private double speed;
    public PID(double newkP,double newkI,double newkD,double newMaxI){
        kP =newkP;
        kI = newkI;
        kD = newkD;
        MaxI = newMaxI;
    }
    double goal=0;
    double position=0;
    double max_acceleration = 1;
    double max_speed = 1;
    double cummulativeError=0;
    double error=0;
    double previous_speed=0;
    double lastError =0;
  
    public double P(double multiplier){
      return multiplier*(error);
    }
    
    public double I(double multiplier){
        if (!(multiplier * Math.abs(cummulativeError + error) > MaxI)) {
            cummulativeError += error;
        }
        return multiplier*cummulativeError;
    }
    public double D(double multiplier){
      return multiplier*(error-lastError);
    }
    
    public double CalculatePID(double current_position,double current_goal,boolean base){
        goal = current_goal;
        position = current_position;
        if (base){
            if (((goal<=Math.PI) && (position <=Math.PI))||((goal>Math.PI)&&(position>Math.PI))){
                error = goal-position;
            } else{
                if (goal-position>Math.PI){
                    //soma menor que PI
                    error = goal-position;
                } else{
                    //objetivo na direita e posição na esquerda
                    error = position+2*Math.PI-goal;
                }
            }
        }else{
        error = goal-position;
        }

        speed = P(kP)+I(kI)+D(kD);

        if (speed>previous_speed+max_acceleration){
            previous_speed += max_acceleration;
            speed = previous_speed;
        }else if (speed < (previous_speed - max_acceleration)){
            previous_speed -= max_acceleration;
            speed = previous_speed;
        }
        if (Math.abs(speed)>max_speed){
            speed = max_speed*Math.abs(speed)/speed;
        }
      
        lastError = error;
      
        return speed;
    }
}

