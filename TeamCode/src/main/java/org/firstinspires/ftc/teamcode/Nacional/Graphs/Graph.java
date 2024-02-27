package org.firstinspires.ftc.teamcode.Nacional.Graphs;

import android.text.GetChars;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.Nacional.SubSystems.RobotHardware;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Vector;

/**
 *
 * @author carlo
 */
public class Graph {
    public static class Pair {
        double XPos;
        double YPos;
        Pair(double nXPos,double nYPos){
            XPos = nXPos;
            YPos = nYPos;
        }
        double getX(){
            return XPos;
        }
        double getY(){
            return YPos;
        }
        void setX(double X){
            XPos=X;
        }
        void setY(double Y){
            YPos = Y;
        }
    }

    public static double currdist = Double.MAX_VALUE;
    public static Vertex closest = null;
    public static int Blue;

    public Graph(int BlueSide){
        Blue = BlueSide;
    }

    public static Map<String,Vertex> allVertexs = new HashMap<String,Vertex>();

    public static Map<String,TrajectorySequence> allSequences = new HashMap<String,TrajectorySequence>();

    public void resetGraph(){
        allVertexs.clear();
        createGraph();
        //creates all vertexes again
        //make the ones that need to be neighbors
    }
    public void put(Vertex v){
        allVertexs.put(v.id, v);
    }
    public void doubleEdge(Vertex v1,Vertex v2){
        v1.addEdge(v2.id, v2);
        v2.addEdge(v1.id, v2);
        allVertexs.put(v1.id,v1);
        allVertexs.put(v2.id,v2);

    }
    public void addToList(Vertex v){
        allVertexs.put(v.id, v);
    }
    public void singleEdge(Vertex src,Vertex dst){
        src.addEdge(dst.id,dst);
        allVertexs.put(src.id,src);
    }
    public static Vertex getClosestVertex(Vertex curP) {
        currdist = Double.MAX_VALUE;
        closest = new Vertex(9999,9999,"9999");
        allVertexs.forEach((k,v)->{
            double dif = (v.XPos-curP.XPos)*(v.XPos-curP.XPos)+(v.YPos-curP.YPos)*(v.YPos-curP.YPos);
            if (dif<currdist){
                currdist = dif;
                closest = v;
            }
            //System.out.println(k+currdist);
        });


        return closest;
    }

    public static Vertex getTarget(Vertex position){
        //System.out.println("printing the target position");
        if (Blue>0){
            if (position.XPos<= -12){
                //System.out.println("BackDropB");
                return allVertexs.get("BackdropB");
            } else{
                //System.out.println("collectBlue");
                return allVertexs.get("collectBlue");
            }
        } else{
            if (position.XPos<= -12){
                //System.out.println("BackDropB");
                return allVertexs.get("BackdropR");
            } else{
                //System.out.println("collectBlue");
                return allVertexs.get("collectRed");
            }
        }
    }
    public static TrajectorySequence getLazySequence(){
        Pose2d currentPose = RobotHardware.autodrive.getPoseEstimate();
        Vertex currentPosition = new Vertex(currentPose.getX(),currentPose.getY(),"currentPosition");
                Vertex closestVertex = getClosestVertex(currentPosition);
        return getFunction(closestVertex.id);
    }

    public static double calculateHeuristic(Vertex currentNode, Vertex target){
        return Math.sqrt((target.XPos-currentNode.XPos)*(target.XPos-currentNode.XPos)+(target.YPos-currentNode.YPos)*(target.YPos-currentNode.YPos));
    }
    public static Vertex aStar(Vertex start, Vertex target){
        PriorityQueue<String> closedList = new PriorityQueue<>();
        PriorityQueue<String> openList = new PriorityQueue<>();
        start.g=0;
        start.f = start.g + calculateHeuristic(start, target);
        openList.add(start.id);

        while(!openList.isEmpty()){
            Vertex n = allVertexs.get(openList.peek());
            if(n == allVertexs.get(target.id)){
                return n;
            }
            for (String nei :n.neighbors){
                Vertex m = allVertexs.get(nei);
                double totalWeight = n.g + Math.sqrt((n.XPos-m.XPos)*(n.XPos-m.XPos)+(n.YPos-m.YPos)*(n.YPos-m.YPos));

                if(!openList.contains(m.id) && !closedList.contains(m.id)){
                    //System.out.println("parent "+ n.id+" son" +m.id);
                    m.parent = n;
                    m.g = totalWeight;
                    m.f = m.g + calculateHeuristic(m,target);
                    openList.add(m.id);
                } else {
                    if(totalWeight < m.g){
                        m.parent = n;
                        m.g = totalWeight;
                        m.f = m.g + calculateHeuristic(m,target);

                        if(closedList.contains(m.id)){
                            closedList.remove(m.id);
                            openList.add(m.id);
                        }
                    }
                }
            };

            openList.remove(n.id);
            closedList.add(n.id);
        }
        return null;
    }

    public static TrajectorySequence recalculateRoute(List<ManualObstacleDetector.Obstacle> obstacleList,Vertex position,Vertex startingPosition){

        Vertex current = getClosestVertex(position);
        Vertex goal = getTarget(startingPosition);
        double curH = Double.MAX_VALUE;
        String next = "NONE";
        for (String neigh: current.neighbors){
            double dif = calculateHeuristic(current,allVertexs.get(neigh)) + calculateHeuristic(allVertexs.get(neigh),goal);
            if (dif<curH){
                curH = dif;
                next = neigh;

            }
            //System.out.println("CurH "+ curH);
        }
        if (allVertexs.get(next)==null){
            return null;
        }
        double deltaX = allVertexs.get(next).XPos-current.XPos;
        double deltaY = allVertexs.get(next).YPos-current.YPos;
        boolean detectionWasUseful =false;

        if (Math.abs(deltaX)>Math.abs(deltaY)&& (obstacleList.contains(ManualObstacleDetector.Obstacle.UP)||obstacleList.contains(ManualObstacleDetector.Obstacle.DOWN))){
            if (deltaX>=0&& obstacleList.contains(ManualObstacleDetector.Obstacle.UP)){
                //obstacle up and movement up
                detectionWasUseful = true;
                System.out.println("U");
            } else if (deltaX<=0&&obstacleList.contains(ManualObstacleDetector.Obstacle.DOWN)){
                //obstacle down and movement down
                detectionWasUseful = true;
                System.out.println("D");
            }
        } else {
            //obstacle is to left or right
            if (deltaY>0 && obstacleList.contains(ManualObstacleDetector.Obstacle.LEFT)){
                //obstacle left and movement left
                System.out.println("L");
                detectionWasUseful = true;
            } else if (deltaY<=0&&obstacleList.contains(ManualObstacleDetector.Obstacle.RIGHT)){
                //obstacle right and movement right
                detectionWasUseful = true;
                System.out.println("R");
            }
        }

        if (detectionWasUseful){
            //System.out.println("next visited vertex is: "+next);
            allVertexs.forEach((k,v)->{
                v.parent = null;
                v.f = 0;
                v.g = 0;
            });
            //System.out.println(allVertexs.get("centerBack").neighbors);
            current.neighbors.remove(next);
            allVertexs.put(current.id,current);
            //System.out.println(allVertexs.get("centerBack").neighbors);

            return printPath(current,goal);
        }
        return null;
    }

    public static TrajectorySequence printPath(Vertex currentP, Vertex target /*,SampleMecanumDrive autodrive*/){
        Vertex start = getClosestVertex(currentP);
        //System.out.println("printing first node visited (START)");
        //System.out.println(start.XPos+" "+ start.YPos);


        Vertex n = aStar(start, target);
        if(n==null){
            //System.out.println(allVertexs.get("centerBack").neighbors);
            System.out.println("NODE N RETURNED = NULL");
            return null;
        }
        List<Pair> Coords = new ArrayList<Pair>();

        System.out.println("printing path taken");
        while(n.parent != null){
            Pair Coord = new Pair(n.XPos,n.YPos);
            Coords.add(Coord);

            n = n.parent;
        }

        Pair Coord = new Pair(n.XPos,n.YPos);
        Coords.add(Coord);

        Collections.reverse(Coords);
        int index=0;
        double heading = 69420;

        System.out.println("lineToLinearHeading("+Coord.XPos+" "+ Coord.YPos+")");


        TrajectorySequenceBuilder path = RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate());

        //path.lineToLinearHeading(new Pose2d(n.XPos,n.YPos,Math.toRadians(0)));//this is fine
        for(Pair coord : Coords){

            if (start.XPos>target.XPos){
            path = path.splineToConstantHeading(new Vector2d(coord.getX(),coord.getY()),Math.toRadians(180));
            heading = 180;
            } else{
            path = path.splineToConstantHeading(new Vector2d(coord.getX(),coord.getY()),Math.toRadians(0));
            heading = 0;
            }

           /*System.out.println("splineToConstantHeading(new Pose2d(" +
                    Coords.get(index).getX()+
                    ","+
                    Coords.get(index).getY()+
                    " ),"+ heading+ ")   ");/**/
        index++;
        }
        //System.out.println("");
        return path.build();
    }


    public void createGraph(){


        //backstage nodes
        Vertex centerBack = new Vertex(30,0,"centerBack");
        Vertex BridgeBackBL = new Vertex(13,60,"BridgeBackBL");
        Vertex BridgeBackBR = new Vertex(13,36,"BridgeBackBR");
        Vertex BridgeBackCL = new Vertex(13,12,"BridgeBackCL");
        Vertex BridgeBackCR = new Vertex(13,-12,"BridgeBackCR");
        Vertex BridgeBackRL = new Vertex(13,-36,"BridgeBackRL");
        Vertex BridgeBackRR = new Vertex(13,-60,"BridgeBackRR");




        //front side nodes
        Vertex centerFront = new Vertex(-50,0,"centerFront");
        Vertex BridgeFrontBL = new Vertex(-37,60,"BridgeFrontBL");
        Vertex BridgeFrontBR = new Vertex(-37,36,"BridgeFrontBR");

        Vertex BridgeFrontCL = new Vertex(-37,12,"BridgeFrontCL");
        Vertex BridgeFrontCR = new Vertex(-37,-12,"BridgeFrontCR");


        Vertex BridgeFrontRL = new Vertex(-37,-36,"BridgeFrontRL");

        Vertex BridgeFrontRR = new Vertex(-37,-60,"BridgeFrontRR");

        addToList(centerBack);
        addToList(BridgeBackBL);
        addToList(BridgeBackBR);
        addToList(BridgeBackCL);
        addToList(BridgeBackCR);
        addToList(BridgeBackRL);
        addToList(BridgeBackRR);
        //addition of all common nodes to the allVertexs list
        //allVertexs.this.put(key, closest)
        //put(centerBack);

        addToList(centerFront);
        addToList(BridgeFrontBL);
        addToList(BridgeFrontBR);
        addToList(BridgeFrontCL);
        addToList(BridgeFrontCR);
        addToList(BridgeFrontRL);
        addToList(BridgeFrontRR);







        //ADD EXCLUSIVE BLUE SIDE CONNECTIONS AND NODES
        if (Blue>0){
            Vertex BackdropB = new Vertex(48,36,"BackdropB");
            Vertex behindBBack = new Vertex(36,36,"behindBBack");
            Vertex collectBlue = new Vertex(-60,-64,"collectBlue");
            Vertex BehindBCollect = new Vertex(-50,-64,"BehindBCollect");

            addToList(BackdropB);
            addToList(behindBBack);


            addToList(BehindBCollect);
            addToList(collectBlue);
        }
        else{

            //ADD EXCLUSIVE RED SIDE CONNECTIONS AND NODES
            Vertex BackdropR = new Vertex(48,-36,"BackdropR");
            Vertex behindRBack = new Vertex(36,-36,"behindRBack");
            Vertex collectRed = new Vertex(-60,64,"collectRed");
            Vertex BehindRCollect = new Vertex(-50,64,"BehindRCollect");

            addToList(BackdropR);

            addToList(behindRBack);


            addToList(BehindRCollect);
            addToList(collectRed);

        }

        //MAKE THE COMMON CONNECTIONS (EDGES BETWEEN VERTEXES)


        /**/
    }
    public static TrajectorySequence getFunction(String VertexID){
        if (VertexID=="centerBack"){

            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(13,-60*Blue,Math.toRadians(180)),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-45,-60*Blue),Math.toRadians(180))
                    .build();

        } else if (VertexID=="BridgeBackBL"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(13,60,Math.toRadians(180)),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-40,60),Math.toRadians(180))
                    .lineToConstantHeading(new Vector2d(-45,-60*Blue))
                    .build();
        } else if (VertexID=="BridgeBackBR"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(13,36,Math.toRadians(180)),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-40,36),Math.toRadians(180))
                    .lineToConstantHeading(new Vector2d(-45,-60*Blue))
                    .build();
        } else if (VertexID=="BridgeBackCL"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(30,0,Math.toRadians(180)),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(13,-60*Blue),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-45,-60*Blue),Math.toRadians(180))
                    .build();
        } else if (VertexID=="BridgeBackCR"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(30,0,Math.toRadians(180)),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(13,-60*Blue),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-45,-60*Blue),Math.toRadians(180))
                    .build();
        } else if (VertexID=="BridgeBackRL"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(13,-36,Math.toRadians(180)),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-40,-36),Math.toRadians(180))
                    .lineToConstantHeading(new Vector2d(-45,-60*Blue))
                    .build();
        }else if (VertexID=="BridgeBackRR"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(13,-60,Math.toRadians(180)),Math.toRadians(180))
                    .splineToConstantHeading(new Vector2d(-40,-60),Math.toRadians(180))
                    .lineToConstantHeading(new Vector2d(-45,-60*Blue))
                    .build();
        }

        else if (VertexID=="centerFront"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(-40,36*Blue,Math.toRadians(180)),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(13,36*Blue),Math.toRadians(0))
                    .build();
        } else if (VertexID=="BridgeFrontBL"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(-40,60,Math.toRadians(180)),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(13,60),Math.toRadians(0))

                    .splineToConstantHeading(new Vector2d(36,36*Blue),Math.toRadians(0))
                    .build();
        } else if (VertexID=="BridgeFrontBR"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(-37,36,Math.toRadians(180)),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(13,36),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(36,36*Blue),Math.toRadians(0))
                    .build();
        } else if (VertexID=="BridgeFrontCL"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(-40,12,Math.toRadians(180)),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(13,12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(36,36*Blue),Math.toRadians(0))
                    .build();

        } else if (VertexID=="BridgeFrontCR"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(-40,-12,Math.toRadians(180)),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(13,-12),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(36,36*Blue),Math.toRadians(0))
                    .build();

        } else if (VertexID=="BridgeFrontRL"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(-40,-36,Math.toRadians(180)),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(13,-36),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(36,36*Blue),Math.toRadians(0))
                    .build();

        }else if (VertexID=="BridgeFrontRR"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(-40,-60,Math.toRadians(180)),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(13,-60),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(36,36*Blue),Math.toRadians(0))
                    .build();
        }

        else  if (VertexID=="BackdropB"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(13,36,Math.toRadians(180)),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-45,36),Math.toRadians(0))
                    .build();

        } else if (VertexID=="behindBBack"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(13,36,Math.toRadians(180)),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-45,36*Blue),Math.toRadians(0))
                    .build();

        } else if (VertexID=="BehindBCollect"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(-40,-12,Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(13,12),Math.toRadians(30))
                    .splineToConstantHeading(new Vector2d(36,36),Math.toRadians(0))
                    .build();

        } else if (VertexID=="collectBlue"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(-40,-12,Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(13,12),Math.toRadians(30))
                    .splineToConstantHeading(new Vector2d(36,36),Math.toRadians(0))
                    .build();

        }


        else  if (VertexID=="BackdropR"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(13,-36,Math.toRadians(180)),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-45,-36),Math.toRadians(0))
                    .build();

        } else if (VertexID=="behindRBack"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .splineToLinearHeading(new Pose2d(13,-36,Math.toRadians(180)),Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(-45,-36),Math.toRadians(0))
                    .build();

        } else if (VertexID=="BehindRCollect"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(-40,12,Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(13,-12),Math.toRadians(30))
                    .splineToConstantHeading(new Vector2d(36,-36),Math.toRadians(0))
                    .build();

        } else if (VertexID=="collectRed"){
            return RobotHardware.autodrive.trajectorySequenceBuilder(RobotHardware.autodrive.getPoseEstimate())
                    .lineToSplineHeading(new Pose2d(-40,12,Math.toRadians(180)))
                    .splineToConstantHeading(new Vector2d(13,-12),Math.toRadians(30))
                    .splineToConstantHeading(new Vector2d(36,-36),Math.toRadians(0))
                    .build();

        }
        return null;
    }

}
