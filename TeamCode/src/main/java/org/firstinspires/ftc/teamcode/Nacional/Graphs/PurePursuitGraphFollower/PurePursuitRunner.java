/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package org.firstinspires.ftc.teamcode.Nacional.Graphs.PurePursuitGraphFollower;
import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Nacional.SubSystems.DriveBase;
import org.firstinspires.ftc.teamcode.Nacional.SubSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Nacional.Utility.Pair;

/**
 *
 * @author carlo
 */
public class PurePursuitRunner {
    int lastIndex;
    public int targetInd;
    public State state= new State();
    PurePursuit pp= new PurePursuit();
    List<Pair> Coords = new ArrayList<>();
    public TargetCourse trajectory;
    public void start(List<Pair> Coords){

        this.trajectory = pp.makeTargetCourse(Coords);
        //trajectory = pp.makeTargetCourse(Coords);
        state.x= RobotHardware.autodrive.getPoseEstimate().getX();;// put pose estivate values
        state.y=RobotHardware.autodrive.getPoseEstimate().getY();;
        state.vx=0;
        state.vy=0;
        this.lastIndex = Coords.size()-1;
        this.targetInd = trajectory.SearchTargetIndex(state).index;
       
        
    }
    public boolean loop(Telemetry telemetry){
        boolean running = this.lastIndex<this.targetInd;;
        if (trajectory.ind+1>=trajectory.cx.size()){
            return false;
        }
        state.x= RobotHardware.autodrive.getPoseEstimate().getX();;// put pose estivate values
        state.y=RobotHardware.autodrive.getPoseEstimate().getY();;

        AccelVector velGoal = pp.targetSpeedControler(state, this.trajectory, this.lastIndex);
        AccelVector actualAccel = pp.PIDControl(velGoal.ax,velGoal.ay, state);
        state.update(actualAccel.ax,actualAccel.ay);
        DriveBase.runMotorsXY(state.vx,state.vy);


        telemetry.addData("state x",state.x);
        telemetry.addData("state y ",state.y);
        telemetry.addData("goal X", trajectory.cx.get(trajectory.ind));
        telemetry.addData("goal Y",trajectory.cy.get(trajectory.ind));
        telemetry.addData("speedx", state.vx);
        telemetry.addData("speedy",state.vy);
        telemetry.addData("ind",trajectory.ind);
        
        return running;
    }
    public void brick(){
        //bricks (stops) robots pp
    }
    
}
