package org.firstinspires.ftc.teamcode.Nacional.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Nacional.Graphs.Graph;
import org.firstinspires.ftc.teamcode.Nacional.Graphs.ManualObstacleDetector;
import org.firstinspires.ftc.teamcode.Nacional.Graphs.Vertex;
import org.firstinspires.ftc.teamcode.Nacional.Utility.PID;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.oldcode.Node;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Config
public class DriveBase {
    public static double BASE_DRIVING_SPEED = 0.5;
    public static double BACKDROP_MULTIPLIER =0.3;
    public static double HANG_SPEED_MULTIPLIER=0.25;
    public static double Gturn=0;
    static YawPitchRollAngles orientation;
    static double CurrentAngle;
    static double CurrentFront;
    public static PID turnPID = new PID(0,0,0,0);
    private static double currentStateMultiplier;
    static Vertex startPosition;



    public static void startGraphMode(boolean BLUESIDE){
        Graph graph = new Graph(BLUESIDE);
        graph.resetGraph();

        startPosition = new Vertex(RobotHardware.autodrive.getPoseEstimate().getX(),RobotHardware.autodrive.getPoseEstimate().getY(),"startPosition");



        Vertex target = Graph.getTarget(startPosition);

        TrajectorySequence sequence = Graph.printPath(startPosition,target);
        RobotHardware.autodrive.followTrajectorySequenceAsync(sequence);
        RobotHardware.autodrive.update();

    }
    public static boolean loopMoveGraph(boolean up, boolean down, boolean left, boolean right,double heading){

        TrajectorySequence newsequence = Graph.recalculateRoute(ManualObstacleDetector.CheckDetections(up,down,left,right,heading),
                new Vertex(RobotHardware.autodrive.getPoseEstimate().getX(),RobotHardware.autodrive.getPoseEstimate().getY(),"Position"),
                startPosition);
        if (newsequence == null) {
            RobotHardware.autodrive.update();
        } else {
            RobotHardware.autodrive.followTrajectorySequenceAsync(newsequence);
            RobotHardware.autodrive.update();
        }
        return RobotHardware.autodrive.isBusy();
    }

    public static void moveWithIMU(double drive,double strafe,double turn,boolean resetIMUorNO,boolean SLOW_MODE){
        RobotHardware.autodrive.update();
        Gturn = turn;
        manageIMU(resetIMUorNO);

        //muda a direção de movimento com base na orientação ro robo
        double rotX = strafe * Math.cos(orientation.getYaw(AngleUnit.RADIANS)) + drive * Math.sin(orientation.getYaw(AngleUnit.RADIANS));
        double rotY = strafe * Math.sin(-orientation.getYaw(AngleUnit.RADIANS)) + drive * Math.cos(-orientation.getYaw(AngleUnit.RADIANS));

        //define as velocidades de cada motor
        double maximo = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(turn), 1);

        double fL = (rotY + rotX + turn) / maximo;
        double fR = (rotY - rotX - turn) / maximo;
        double bL = (rotY - rotX + turn) / maximo;
        double bR = (rotY + rotX - turn) / maximo;
        /**/
        /*
        geração de uma aceleração ao controlar o maximo dos motores
        aplica as potências aos motores
        */
        currentStateMultiplier = BASE_DRIVING_SPEED;
        if ((/*RobotHardware.distance.getDistance(DistanceUnit.CM)<30 &&(ArmMovement.currentArmState == ArmMovement.ARM_STATE.DEPOSIT_BACK))||
        */SLOW_MODE)){
            currentStateMultiplier = BACKDROP_MULTIPLIER;
        } else if (HangRobot.currentDesiredHeight>=HangRobot.HOOK_HEIGHT_HANG){
            currentStateMultiplier = HANG_SPEED_MULTIPLIER;
        }

        RobotHardware.setDrivebasePower(fL,fR,bL,bR,currentStateMultiplier);

    }
    public static void moveBaseGraphMode(){
        //TODO get this all right
    }
    public static void manageIMU(boolean resetIMUorNO){
        orientation = RobotHardware.imu.getRobotYawPitchRollAngles();
        //DeltaAngle =360- (orientation.getYaw(AngleUnit.RADIANS)*180/3.141);
        if(resetIMUorNO){
            RobotHardware.imu.resetYaw();
            CurrentFront = orientation.getYaw(AngleUnit.RADIANS);
        }

        //ao girar manualmente, reseta a orientação tida como frente pelo robô (para uso no PID)
        if (Math.abs(Gturn)>=0.001){
            CurrentFront = orientation.getYaw(AngleUnit.RADIANS);
            turnPID.cummulativeError=0;

        }



        //Parametros para o PID são calculados (não é tudo feito dentro do PID para permitir uso na telemetria)
        CurrentAngle = orientation.getYaw(AngleUnit.RADIANS);
        if (CurrentAngle <CurrentFront){
            CurrentAngle = 2*Math.PI+CurrentAngle-CurrentFront;
        } else{
            CurrentAngle-=CurrentFront;
        }

        //se não houver uma curva manual, um PID é usado para manter a orientação do robô
        if (Math.abs(Gturn)<0.001){
            Gturn = turnPID.CalculatePID(ConvertAngle(CurrentAngle),0,true)*-1;
            if (Math.abs(Gturn)<0.1){
                Gturn =0;
            }
        }
    }

    /**
    converts an angle based on front, used to control heading lock pid
     */
    public static double ConvertAngle(double angle){
        double return_angle = angle;
        if(angle>Math.PI){
            return_angle -= 2*Math.PI;
        }
        return return_angle;
    }



}
