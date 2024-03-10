/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package org.firstinspires.ftc.teamcode.Nacional.Graphs.PurePursuitGraphFollower;
import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.Nacional.SubSystems.DriveBase;
import org.firstinspires.ftc.teamcode.Nacional.SubSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.Nacional.Utility.Pair;

/**
 *
 * @author carlo
 */
public class PurePursuitRunner {
    int lastIndex;
    int targetInd;
    State state= new State();;
    PurePursuit pp= new PurePursuit();
    List<Pair> Coords = new ArrayList<>();
    TargetCourse trajectory;
    public void start(List<Pair> Coords){
        this.trajectory = pp.makeTargetCourse(Coords);
        //trajectory = pp.makeTargetCourse(Coords);
        state.x= RobotHardware.autodrive.getPoseEstimate().getX();;// put pose estivate values
        state.y=RobotHardware.autodrive.getPoseEstimate().getY();;
        this.lastIndex = Coords.size()-1;
        this.targetInd = trajectory.SearchTargetIndex(state).index;
       
        
    }
    public boolean loop(){
        boolean running = this.lastIndex<=this.targetInd;;
        AccelVector accelGoal = pp.targetSpeedControler(state, this.trajectory, this.lastIndex);
        AccelVector actualAccel = pp.PIDControl(accelGoal.ax,accelGoal.ay, state);
        state.update(actualAccel.ax,actualAccel.ay);
        DriveBase.runMotorsXY(state.vx,state.vy);
        
        return running;
    }
    public void brick(){
        //bricks (stops) robots pp
    }
    
}
