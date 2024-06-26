package org.firstinspires.ftc.teamcode.Nacional.SubSystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Nacional.Graphs.Graph;
import org.firstinspires.ftc.teamcode.Nacional.Graphs.ManualObstacleDetector;
import org.firstinspires.ftc.teamcode.Nacional.Graphs.PurePursuitGraphFollower.PurePursuitRunner;
import org.firstinspires.ftc.teamcode.Nacional.Graphs.Vertex;
import org.firstinspires.ftc.teamcode.Nacional.Teleop.DuoBlue;
import org.firstinspires.ftc.teamcode.Nacional.Utility.PID;
import org.firstinspires.ftc.teamcode.Nacional.Utility.Pair;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@Config
public class DriveBase {
    public static double BASE_DRIVING_SPEED = 0.7;
    public static double BACKDROP_MULTIPLIER =0.5;
    public static double HANG_SPEED_MULTIPLIER=0.5;
    public static double Gturn=0;
    static YawPitchRollAngles orientation;
    static double CurrentAngle;
    public static double CurrentFront;
    public static PID turnPID = new PID(0.2,0,0,0);
    public static double KP = 0.0;
    public static PID turnStrongPID = new PID(0.8,0,0,0);
    private static double currentStateMultiplier;
    public static double CurrentDistTo0;

    static Vertex startPosition;
    //public Graph graph;
    public static PurePursuitRunner pp = new PurePursuitRunner();
    //public void initGraph(int BLUESIDE){
       // graph = new Graph(BLUESIDE);
       // graph.resetGraph();
    //}



    public static void startGraphMode(int BLUESIDE){
        Graph.Blue = BLUESIDE;
        Graph.resetGraph();
        RobotHardware.moveArm(0);
        startPosition = new Vertex(RobotHardware.autodrive.getPoseEstimate().getX(),RobotHardware.autodrive.getPoseEstimate().getY(),"startPosition");
        Vertex target = Graph.getTarget(startPosition);
        //List<Pair> fart = new ArrayList<>();
        //fart.add(new Pair(RobotHardware.autodrive.getPoseEstimate().getX(),RobotHardware.autodrive.getPoseEstimate().getY()));
        //fart.add(new Pair(30,0));
        //fart.add(new Pair(36,-36));
        //fart.add(new Pair(-50,-36));
        //return fart;
        //pp.start(Graph.recalculateRoute(new ArrayList<Ma
        // nualObstacleDetector.Obstacle>(), startPosition,startPosition));
        pp.start(Graph.printPath(startPosition,target));




    }
    public static boolean loopMoveGraph(boolean up, boolean down, boolean left, boolean right, double heading, boolean exit, Telemetry telemetry){
        if (exit){
            DuoBlue.currentMode = DuoBlue.BASE_MODE.NORMAL;
            pp.brick();
            return false;
        }

        return pp.loop(telemetry);
    }

    public static void moveWithIMU(double drive,double strafe,double turn,boolean resetIMUorNO,boolean SLOW_MODE){
        //RobotHardware.autodrive.update();
        turnPID.kP= KP;
        Gturn= turn;
        manageIMU(resetIMUorNO);

        //muda a direção de movimento com base na orientação ro robo
        double rotX = strafe * Math.cos(CurrentFront) + drive * Math.sin(CurrentFront);
        double rotY = strafe * Math.sin(-CurrentFront) + drive * Math.cos(-CurrentFront);

        //define as velocidades de cada motor
        double maximo = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(Gturn), 1);

        double fL = (rotY + rotX + Gturn) / maximo;
        double fR = (rotY - rotX - Gturn) / maximo;
        double bL = (rotY - rotX + Gturn) / maximo;
        double bR = (rotY + rotX - Gturn) / maximo;
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
            //RobotHardware.imu.resetYaw();
            CurrentDistTo0 =orientation.getYaw(AngleUnit.RADIANS);
            CurrentFront = orientation.getYaw(AngleUnit.RADIANS) - CurrentDistTo0;
        }

        //ao girar manualmente, reseta a orientação tida como frente pelo robô (para uso no PID)
        if (Math.abs(Gturn)>=0.001){
            CurrentFront = orientation.getYaw(AngleUnit.RADIANS)- CurrentDistTo0;

        }



        //Parametros para o PID são calculados (não é tudo feito dentro do PID para permitir uso na telemetria)
        CurrentAngle = orientation.getYaw(AngleUnit.RADIANS);

        //se não houver uma curva manual, um PID é usado para manter a orientação do robô
        if (Math.abs(Gturn)<0.001){
            Gturn = turnPID.CalculatePID(ConvertAngle(CurrentFront,CurrentAngle),0)*-1;
            if (Math.abs(Gturn)<0.1){
                Gturn =0;
            }
        }
    }

    /**
    converts an angle based on front, used to control heading lock pid
     */
    public static double ConvertAngle(double currentAngle, double desiredAngle){
        double angleError = Math.PI-Math.abs(Math.abs(currentAngle-desiredAngle)-Math.PI);
        if (currentAngle+angleError%(2*Math.PI)!=desiredAngle){
            angleError = - angleError;
        }

        return angleError;
    }

    public static void runMotorsXY(double vx, double vy) {
        double rotX = vy * Math.cos(Math.PI) + vx * Math.sin(Math.PI);
        double rotY = vy * Math.sin(-Math.PI) + vx * Math.cos(-Math.PI);

        double turn =turnStrongPID.CalculatePID(RobotHardware.autodrive.getPoseEstimate().getHeading(),Math.toRadians(180))*-1;

        //define as velocidades de cada motor
        double maximo = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(turn), 1);

        double fL = (rotY - rotX + turn) / maximo;
        double fR = (rotY + rotX - turn) / maximo;
        double bL = (rotY + rotX + turn) / maximo;
        double bR = (rotY - rotX - turn) / maximo;

        RobotHardware.setDrivebasePower(fL,fR,bL,bR,1);
    }
//this cv function should work better for the new code that does not reset the imu
}
