package org.firstinspires.ftc.teamcode.CentersTop;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.CentersTop.PID;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * DriveVariables, everything variables used in the TeleOp mode is here (or supposed to be here)
 */
public class DV {
    public static DcMotor frontRight;
    public static DcMotor backRight;
    public static DcMotor frontLeft;
    public static DcMotor backLeft;
    public static CRServo mainArm;
    public static Servo Wrist;
    public static SampleMecanumDrive autodrive;
    public static double rotX;
    public static double rotY;
    public static double maximo;
    public Servo Arm2;
    public static Servo Articulation;
    public static Servo LClaw;
    public static Servo RClaw;
    public static Servo PlaneLauncher;
    public TouchSensor bottomLimit;
    //public Servo servoArticulacao;
    //public Servo servoGarra;
    public static IMU imu;
    DigitalChannel greenLED;
    DigitalChannel redLED;
    static DistanceSensor distanceSensor;
    static YawPitchRollAngles orientation;
    SampleMecanumDrive Autodrive;
    static double drive=0;
    static double turn=0;
    static double strafe=0;
    double DeltaAngle;
    static double CurrentFront;
    static double CurrentAngle;
    static double fL;
    static double fR;
    static double bL;
    static double bR =0;
    boolean yOn;
    static double powerbraco;
    double finalgoalbraco = 0;
    double goalbraco;
    //PID pid_turn = new PID(0.5,0.005,0.0,0.4);
    static PID pid_turn = new PID(0.6,0.000,0.0,0.3);
    public static int GPVARIATION=6;
    public static int MAXHEIGHT=1700;

    static PID pid_braco = new PID(0.003,0,0,0);
    double prevtime=0;
    double Artpos =0;
    boolean autoGrab = false;
    boolean AutoGrabDone=true;
    boolean IMUWorking = true;
    boolean encodersWorking = true;
    simpleSwitch leftClawSwitch = new simpleSwitch();
    simpleSwitch rightClawSwitch = new simpleSwitch();
    public static enum RunMode{
        FULL_FUNCTION,
        ENCODER_ERROR,
        INCOMPETENT_IMU
    }
    public static enum BaseExecMode{
        FART
    }
    static BaseExecMode currentBaseExecMode = BaseExecMode.FART;
    public class simpleSwitch{
        public boolean previouslyPressed=false;
        public boolean currentState = false;
        boolean click(boolean pressed) {
            if (!previouslyPressed && pressed) {
                currentState = !currentState;
            }
            previouslyPressed = pressed;
            return currentState;
        }
    }
}
