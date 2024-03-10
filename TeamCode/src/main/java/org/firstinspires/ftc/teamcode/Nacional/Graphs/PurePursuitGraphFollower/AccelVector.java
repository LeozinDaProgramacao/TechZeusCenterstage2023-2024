/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */
package org.firstinspires.ftc.teamcode.Nacional.Graphs.PurePursuitGraphFollower;

/**
 *
 * @author carlo
 */
class AccelVector{
        double ax, ay;
        int index;
        public void setAccel(double nax,double nay){
            this.ax = nax;
            this.ay = nay;
        }
        public void setInd(int index){
            this.index = index;
        }
        public int getIndex(){
            return this.index;
        }
        public double getAX(){
            return this.ax;
        }
        public double getAY(){
            return this.ay;
        }
   }
