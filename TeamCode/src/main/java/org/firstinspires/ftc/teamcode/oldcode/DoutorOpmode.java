package org.firstinspires.ftc.teamcode.oldcode;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Doutor Opmode")
@Disabled
public class DoutorOpmode extends LinearOpMode {
    ElapsedTime elapsedTime = new ElapsedTime();

    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor mainArm;

    private  Servo Articulation;
    private Servo LClaw;
    private Servo RClaw;
    private Servo PlaneLauncher;
    private TouchSensor bottomLimit;

    //private Servo servoArticulacao;
    //private Servo servoGarra;
    private IMU imu;
    YawPitchRollAngles orientation;
    SampleMecanumDrive Autodrive;
    double drive=0;
    double turn=0;
    double strafe=0;
    double raw_drive=0;
    double raw_turn=0;
    double raw_strafe=0;
    double DeltaAngle;
    double CurrentFront;
    double CurrentAngle;
    double fL=0;
    double fR=0;
    double bL=0;
    double bR=0;
    double DfL=0;
    double DfR=0;
    double DbL=0;
    double DbR=0;
    boolean yOn;
    double powerbraco;
    double finalgoalbraco = 0;
    double goalbraco;
    OpenCvWebcam webcam;
    //PID pid_turn = new PID(0.5,0.005,0.0,0.4);
    PID pid_turn = new PID(0.6,0.000,0.0,0.3);
    //PID pid_braco = new PID(0.005,0.01,0.003);
    double prevtime=0;
    double Artpos =0;
    boolean autoGrab = false;
    boolean AutoGrabDone=true;
    double startingIMU;
    double startFLencoder;
    double startFRencoder;
    double startBLencoder;
    double startBRencoder;

    /**
     * This function is executed when this Op Mode is selected.
     */
    SampleMecanumDrive autodrive;
    @Override
    public void runOpMode() {
        autodrive = new SampleMecanumDrive(hardwareMap);
        //inicializa hardware
        imu = hardwareMap.get(IMU.class, "imu");
        frontRight = hardwareMap.get(DcMotor.class, "FRmotor");
        backRight = hardwareMap.get(DcMotor.class, "BRmotor");
        frontLeft = hardwareMap.get(DcMotor.class, "FLmotor");
        backLeft = hardwareMap.get(DcMotor.class, "BLmotor");
        mainArm = hardwareMap.get(DcMotor.class, "MainArm");
        bottomLimit= hardwareMap.get(TouchSensor.class,"bottomLimitSensor");

        Articulation = hardwareMap.get(Servo.class,"Articulation");
        LClaw = hardwareMap.get(Servo.class,"LClaw");
        RClaw = hardwareMap.get(Servo.class,"RClaw");
        PlaneLauncher = hardwareMap.get(Servo.class,"PlaneLauncher");



        //corrige orientação dos motores
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        //falsifica um encoder pro braço

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */
        //servoArticulacao.setDirection(DcMotorSimple.Direction.REVERSE);


        //define variaveis essenciais e inicializa funções


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        BlueDetector detector = new BlueDetector(telemetry);
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

        YawPitchRollAngles orientation = null;
        AngularVelocity angularVelocity;
        PlaneLauncher.setPosition(0.5);
        telemetry.addData("Webcam Status","To check webcam status click on the three dots in the top right and select 'camera stream'");
        telemetry.update();
        waitForStart();
        webcam.stopStreaming();

        double CurrentFront = 0;
        double startingIMU = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double startFLencoder = frontLeft.getCurrentPosition();
        double startFRencoder = frontLeft.getCurrentPosition();
        double startBLencoder = frontLeft.getCurrentPosition();
        double startBRencoder = frontLeft.getCurrentPosition();

        while  (opModeIsActive()) {
            braco();
            garra();
            inputs();
            manageIMU();
            moveBase();
            adicionarTelemetria();
        }
    }
    private void inputs(){
        //get gamepad axis
        if (gamepad1.left_bumper){
            drive = gamepad1.left_stick_y*-1*0.4;
            turn = gamepad1.right_stick_x*0.4;
            strafe = gamepad1.left_stick_x*0.4;
        } else {
            drive = gamepad1.left_stick_y * -1;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;
        }


        if (!autoGrab&&gamepad1.right_bumper){
            AutoGrabDone=false;
        }
        autoGrab= gamepad1.right_bumper;

        orientation = imu.getRobotYawPitchRollAngles();
        DeltaAngle =360- (orientation.getYaw(AngleUnit.RADIANS)*180/3.141);
    }
    private void manageIMU(){

        if(gamepad1.a){
            imu.resetYaw();
            CurrentFront = orientation.getYaw(AngleUnit.RADIANS);
        }

        //ao girar manualmente, reseta a orientação tida como frente pelo robô (para uso no PID)
        if (Math.abs(turn)>=0.001){
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
        if (Math.abs(turn)<0.001){
            turn = pid_turn.CalculatePID(ConvertAngle(CurrentAngle),0,true)*-1;
            if (Math.abs(turn)<0.1){
                turn =0;
            }
        }
    }
    private void moveBase(){
        if (!autoGrab&&AutoGrabDone){
            //muda a direção de movimento com base na orientação ro robo
            double rotX = strafe*Math.cos(orientation.getYaw(AngleUnit.RADIANS))+drive*Math.sin(orientation.getYaw(AngleUnit.RADIANS));
            double rotY = strafe*Math.sin(-orientation.getYaw(AngleUnit.RADIANS))+drive*Math.cos(-orientation.getYaw(AngleUnit.RADIANS));

            //define as velocidades de cada motor
            double maximo = Math.max(Math.abs(rotX)+Math.abs(rotY)+Math.abs(turn),1);


            fL= (rotY+rotX+turn)/maximo;
            fR=(rotY-rotX-turn)/maximo;
            bL=(rotY-rotX+turn)/maximo;
            bR=(rotY+rotX-turn)/maximo;
        /*
        fL= (rotY+rotX+turn)/maximo;
        fR=(rotY-rotX-turn)/maximo;
        bL=(rotY-rotX+turn)/maximo;
        bR=(rotY+rotX-turn)/maximo;
        /**/
            //geração de uma aceleração ao controlar o maximo dos motores
            //aplica as potências aos motores

            frontRight.setPower(fR*0.5);
            frontLeft.setPower(fL*0.5);
            backRight.setPower(bR*0.5);
            backLeft.setPower(bL*0.5);/**/}
        else {
            autodrive.setPoseEstimate(new Pose2d(0,0,0));
            Artpos = 0.71;
            Articulation.setPosition(0.71);
            autodrive.followTrajectory(autodrive.trajectoryBuilder(new Pose2d(0,0,0))
                    .back(2)
                    .build());
            AutoGrabDone=true;
        }

    }
    private void adicionarTelemetria(){
      /*Description:
      this funtion adds all debugging data needed to the telemetry board
      */
        if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)!=startingIMU){telemetry.addData("IMU Status","Functional"+imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));} else {telemetry.addData("IMU Status","Not Functional");}

        if (frontLeft.getCurrentPosition()!=startFLencoder){
            telemetry.addData("FL Encoder Status","Functional");
        } else {
            telemetry.addData("FL Encoder Status","Not Functional");
        }

        if (backLeft.getCurrentPosition()!=startBLencoder){
            telemetry.addData("BL Encoder Status","Functional");
        } else {
            telemetry.addData("BL Encoder Status","Not Functional");
        }
        if (frontRight.getCurrentPosition()!=startFRencoder){
            telemetry.addData("FR Encoder Status","Functional");
        } else {
            telemetry.addData("FR Encoder Status","Not Functional");
        }
        if (backRight.getCurrentPosition()!=startBRencoder){
            telemetry.addData("FR Encoder Status","Functional");
        } else {
            telemetry.addData("FR Encoder Status","Not Functional");
        }
        telemetry.addData("Articulation Servo Status","is functional if pressing upped dpad and holding right stick up leaves it paralel to the floor");
        telemetry.addData("Claw Servos Statur","if they fully open, they are functional, else they are attached incorrectly to tthe claw");
        telemetry.addData("Bottom Limit Sensor Status (if it changes uppon pressing, it works)Current Status",bottomLimit.isPressed());
        telemetry.addData("Airplane Servo Status","press the top left button or top right button on gamepad2, if it moves, it works");

        telemetry.update(); // adiciona tudo da telemetria
    }
    private void braco() {

        if (gamepad2.left_bumper){
            PlaneLauncher.setPosition(0);
        }
        if (gamepad2.right_bumper){
            PlaneLauncher.setPosition(1);
        }
        if (Math.abs(gamepad2.right_trigger)>Math.abs(gamepad2.left_trigger)){
            mainArm.setPower(gamepad2.right_trigger*0.4);
        }
        else {
            if (!bottomLimit.isPressed()) {
                mainArm.setPower(-gamepad2.left_trigger*0.4);
            } else{
                mainArm.setPower(0.07);
            }
        }
    }
    private void garra(){
        if (gamepad2.x) {
            openClaw();
        } else if (gamepad2.y) {
            closeClaw();
        }
        if (gamepad2.dpad_down) {
            Artpos = 0.71;
        } else if (gamepad2.dpad_up) {
            Artpos = 0.45;
        } else if (gamepad2.dpad_left) {
            Artpos = 0.64;
        } else if (gamepad2.dpad_right){
            Artpos =0;
            closeClaw();
        }
        Articulation.setPosition(Artpos + gamepad2.right_stick_y * 0.1);
    }
    private void closeClaw(){
        RClaw.setPosition(0.3);
        LClaw.setPosition(0.7);
    }
    private void openClaw(){
        RClaw.setPosition(0.5);
        LClaw.setPosition(0.5);
    }

    public double ConvertAngle(double angle){
        double return_angle = angle;
        if(angle>Math.PI){
            return_angle -= 2*Math.PI;
        }
        return return_angle;
    }

}
