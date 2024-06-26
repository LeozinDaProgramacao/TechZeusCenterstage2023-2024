/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 */
package org.firstinspires.ftc.teamcode.Nacional.Graphs.PurePursuitGraphFollower;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import org.firstinspires.ftc.teamcode.Nacional.Utility.Pair;
/**
 *
 * @author carlo
 */
public class PurePursuit {

    public AccelVector accel = new AccelVector();

    
    
    public AccelVector PIDControl(double XVTarget,double YVTarget, State state){
        
        double ax = PurePursuitConstants.kP * (XVTarget-state.vx);
        double ay = PurePursuitConstants.kP * (YVTarget-state.vy);
        double div =Math.abs(ax)+Math.abs(ay);
        double rdiv = Math.max(1,div/PurePursuitConstants.MAXACCEL);
        this.accel.setAccel(ax/rdiv, ay/rdiv);
        
        return this.accel;
    }
    
    
    public AccelVector targetSpeedControler(State state,TargetCourse trajectory, int pind){
        PursuitData data = trajectory.SearchTargetIndex(state);
        int tind = data.index;
        double Lf = data.Lf;
        if (pind>tind){
            tind = pind;
            
        }
        double tx = trajectory.cx.get(tind);
        double ty = trajectory.cy.get(tind);
        double alpha = Math.atan2(ty-state.y, tx-state.x);
        
        double xacell = Math.cos(alpha)*PurePursuitConstants.MAXSPEED;
        double yacell = Math.sin(alpha)*PurePursuitConstants.MAXSPEED;
        
        //if (Math.abs(state.vx+xacell)>MAXSPEED){
        //    xacell = 0;
        //}
        //if (Math.abs(state.vy+yacell)>MAXSPEED){
        //    yacell = 0;
        //}
        AccelVector vector = new AccelVector();
        vector.setAccel(xacell,yacell);
        vector.setInd(tind);
        return vector;
    }
    public TargetCourse makeTargetCourse(List<Pair> Coords){
        TargetCourse trajectory = new TargetCourse();
        for (int pos=0;pos<Coords.size()-1;pos++){
            double xDist = (Coords.get(pos+1).getX()-Coords.get(pos).getX())/(PurePursuitConstants.spp-1);
            double yDist = (Coords.get(pos+1).getY()-Coords.get(pos).getY())/(PurePursuitConstants.spp-1);
        
            for (int x = 0; x<PurePursuitConstants.spp-1;x++){
                trajectory.cx.add(Coords.get(pos).getX()+x*xDist);
                trajectory.cy.add(Coords.get(pos).getY()+x*yDist);
            }
        }
        trajectory.cx.add(Coords.get(Coords.size()-1).getX());
        trajectory.cy.add(Coords.get(Coords.size()-1).getY());
        
        System.out.println(trajectory.cx);
        System.out.println(trajectory.cy);
        return trajectory;
    }

    
}
