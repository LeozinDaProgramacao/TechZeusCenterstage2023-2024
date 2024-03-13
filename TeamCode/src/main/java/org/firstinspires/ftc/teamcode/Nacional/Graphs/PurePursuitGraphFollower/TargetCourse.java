/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package org.firstinspires.ftc.teamcode.Nacional.Graphs.PurePursuitGraphFollower;
import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author carlo
 */
public class TargetCourse{
        double Lfc=PurePursuitConstants.Lfc;//look forward distance
        double PPk = PurePursuitConstants.PPk;
        public List<Double> cx= new ArrayList<>();
        public List<Double> cy = new ArrayList<>();
        public int ind;
        
        Integer old_nearest_point;
        List<Double> dist = new ArrayList<>();
        
        
        double distThisInd=Double.MAX_VALUE;
        double distNextInd = Double.MAX_VALUE;
        
        public PursuitData SearchTargetIndex (State state){
            double mindist = Double.MAX_VALUE;
            ind = Integer.MAX_VALUE;
            if (old_nearest_point==null){
                for (int i=0;i<cx.size();i++){
                    double disttoI = Math.hypot(state.x-cx.get(i), state.y-cy.get(i));
                    if (disttoI<mindist){
                        mindist = disttoI;
                        ind = i;
                        old_nearest_point = ind;
                    }
                }
                
            } else{
                ind = old_nearest_point;
                distThisInd = state.calcDist(state,this.cx.get(ind+1), this.cy.get(ind+1));
                
                while (true){
                    if (ind+1 >=this.cx.size()){
                        break;
                    }
                    distNextInd = state.calcDist(state, this.cx.get(ind+1), this.cy.get(ind+1));
                    if (distThisInd<distNextInd){
                        break;
                    }
                    if (ind+1<this.cx.size()){
                        ind++;
                    }
                }
                this.old_nearest_point=ind;
            }
            double Lf = PPk* Math.hypot(state.vx,state.vy)+Lfc;

            if (ind+1>cx.size()){
                PursuitData data = new PursuitData(ind,Lf);
                return data;
            }
            while (Lf > state.calcDist(state,this.cx.get(ind),this.cy.get(ind))){
                if (ind+1>=cx.size()){
                    break;
                }
                ind++;
                
            }
            if (Lf< state.calcDist(state,this.cx.get(ind), this.cy.get(ind))){
                this.old_nearest_point= null;
            }
            PursuitData data = new PursuitData(ind,Lf);
            return data;
        }
    }