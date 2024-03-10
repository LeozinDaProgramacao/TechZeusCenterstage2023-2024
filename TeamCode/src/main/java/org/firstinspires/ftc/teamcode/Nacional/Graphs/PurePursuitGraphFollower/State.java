/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package org.firstinspires.ftc.teamcode.Nacional.Graphs.PurePursuitGraphFollower;

/**
 *
 * @author carlo
 */
class State {
        double x=0;
        double y=0;
        double vx=0;
        double vy=0;
        double yawVec=0;
        
        
        public void update(double ax, double ay){
            this.vx += ax;
            this.vy += ay;
            //this.x += vx;
            //this.y +=vy;
        }
        
        public double calcDist(State state, double point_x, double point_y){
            double dx = state.x-point_x;
            double dy = state.y-point_y;
     
            return(Math.sqrt(dx*dx+dy*dy));
        }
    }
