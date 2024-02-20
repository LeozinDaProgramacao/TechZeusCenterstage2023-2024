package org.firstinspires.ftc.teamcode.Nacional.Graphs;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author carlo
 */
public class ManualObstacleDetector {
    public enum Obstacle{
        UP,DOWN,LEFT,RIGHT;
    }
    static List<Obstacle> detectionList = new ArrayList<Obstacle>();
    public static List<Obstacle> CheckDetections(boolean up,boolean down, boolean left, boolean right,double heading){
        boolean alignedAtFirst = (heading<Math.toRadians(45)||heading>Math.toRadians(135));
        //recieves inputs from all sensors and sees which has been activated bellow the treshold for longest
        // this is then returned as a Obstacle enumeration (up, down, left or roght)

        if (up){detectionList.add(Obstacle.UP);}
        if (down){detectionList.add(Obstacle.DOWN);}
        if (left){detectionList.add(Obstacle.LEFT);}
        if (right){detectionList.add(Obstacle.RIGHT);}
        return detectionList;
    }
}