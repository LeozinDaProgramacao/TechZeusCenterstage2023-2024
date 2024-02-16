/*
 * Click nbfs://nbhost/SystemFileSystem/Templates/Licenses/license-default.txt to change this license
 * Click nbfs://nbhost/SystemFileSystem/Templates/Classes/Class.java to edit this template
 */

package org.firstinspires.ftc.teamcode.oldcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;

/**
 *
 * @author carlo
 */
public class Node implements Comparable<Node> {

    // Id for readability of result purposes
    private static int idCounter = 0;
    public int id;

    // Parent in the path
    public Node parent = null;

    public List<Edge> neighbors;

    // Evaluation functions
    public double f = Double.MAX_VALUE;
    public double g = Double.MAX_VALUE;
    // Hardcoded heuristic
    public double h;
    public double XPos;
    public double YPos;


    Node(double XPos,double YPos){
        this.XPos = XPos;
        this.YPos = YPos;
        this.id = idCounter++;
        this.neighbors = new ArrayList<>();
    }


    @Override
    public int compareTo(Node n) {
        return Double.compare(this.f, n.f);
    }

    public static class Edge {
        Edge(double weight, Node node){
            this.weight = weight;
            this.node = node;
        }

        public double weight;
        public Node node;
    }
    public void addOneDirectionalBranch(Node node){
        Edge newEdge = new Edge(Math.sqrt((node.XPos-this.XPos)*(node.XPos-this.XPos)+(node.YPos-this.YPos)*(node.YPos-this.YPos)), node);
        neighbors.add(newEdge);
    }

    public void addBranch(Node node){
        Edge newEdge = new Edge(Math.sqrt((node.XPos-this.XPos)*(node.XPos-this.XPos)+(node.YPos-this.YPos)*(node.YPos-this.YPos)), node);
        neighbors.add(newEdge);
        Edge newEdgeR = new Edge(Math.sqrt((node.XPos-this.XPos)*(node.XPos-this.XPos)+(node.YPos-this.YPos)*(node.YPos-this.YPos)), this);
        node.neighbors.add(newEdgeR);
    }
    public static Node getClosestNode(Node curP,List<Node> nodeList) {
        double currdist = Double.MAX_VALUE;
        Node closest = null;
        for(Node n : nodeList){
            double dif = Math.sqrt((n.XPos-curP.XPos)*(n.XPos-curP.XPos)+(n.YPos-curP.YPos)*(n.YPos-curP.YPos));
            if (dif<currdist){
                currdist = dif;
                closest = n;
            }
        }
        return closest;
    }
    public static Node getTarget(Node position,List<Node> allNodesList){
        System.out.println("printing the target position");
        if (position.XPos>= -12){
            System.out.println(allNodesList.get(allNodesList.size()-1).XPos+" "+ allNodesList.get(allNodesList.size()-1).YPos);
            return allNodesList.get(allNodesList.size()-1);
        } else{
            System.out.println(allNodesList.get(allNodesList.size()-2).XPos+" "+ allNodesList.get(allNodesList.size()-2).YPos);
            return allNodesList.get(allNodesList.size()-2);
        }
    }

    public static double calculateHeuristic(Node currentNode, Node target){
        return Math.sqrt((target.XPos-currentNode.XPos)*(target.XPos-currentNode.XPos)+(target.YPos-currentNode.YPos)*(target.YPos-currentNode.YPos));
    }

    public static Node aStar(Node start, Node target){
        PriorityQueue<Node> closedList = new PriorityQueue<>();
        PriorityQueue<Node> openList = new PriorityQueue<>();
        start.g=0;
        start.f = start.g + calculateHeuristic(start, target);
        openList.add(start);

        while(!openList.isEmpty()){
            Node n = openList.peek();
            if(n == target){
                return n;
            }

            for(Node.Edge edge : n.neighbors){
                Node m = edge.node;
                double totalWeight = n.g + edge.weight;

                if(!openList.contains(m) && !closedList.contains(m)){
                    m.parent = n;
                    m.g = totalWeight;
                    m.f = m.g + calculateHeuristic(m,target);
                    openList.add(m);
                } else {
                    if(totalWeight < m.g){
                        m.parent = n;
                        m.g = totalWeight;
                        m.f = m.g + calculateHeuristic(m,target);

                        if(closedList.contains(m)){
                            closedList.remove(m);
                            openList.add(m);
                        }
                    }
                }
            }

            openList.remove(n);
            closedList.add(n);
        }
        return null;
    }

    public static TrajectorySequence //replace void with TrajectorySequence or sth like that
    printPath(Node currentP, List<Node> allNodesList , SampleMecanumDriveCancelable autodrive){
        Node start = getClosestNode(currentP, allNodesList);
        System.out.println("printing first node visited (START)");
        System.out.println(start.XPos+" "+ start.YPos);

        Node target = getTarget(currentP,allNodesList);
        Node n = aStar(start, target);
        if(n==null)
            return null;
        List<Pair> Coords = new ArrayList<Pair>();
        Pair Coord=null;
        List<Integer> ids = new ArrayList<>();

        while(n.parent != null){
            Coord = new Pair(n.XPos,n.YPos);
            Coords.add(Coord);
            ids.add(n.id);
            n = n.parent;
        }
        //ids.add(n.id);
        Coord = new Pair(n.XPos,n.YPos);
        //Coords.add(Coord);
        Collections.reverse(Coords);
        Collections.reverse(ids);
        int index=0;
        double heading = 69420;
        System.out.println("lineToLinearHeading("+Coord.XPos+" "+ Coord.YPos+")");
        TrajectorySequenceBuilder path= autodrive.trajectorySequenceBuilder(autodrive.getPoseEstimate());
        path.lineToLinearHeading(new Pose2d(start.XPos,start.YPos,Math.toRadians(0)));
        for(int id : ids){

            if (start.XPos>target.XPos){
                path.splineToConstantHeading( new Vector2d(Coords.get(index).getX(),Coords.get(index).getY()),Math.toRadians(180));
                heading = 180;
            } else{
                path.splineToConstantHeading( new Vector2d(Coords.get(index).getX(),Coords.get(index).getY()),Math.toRadians(0));
                heading = 0;
            }

            System.out.println("splineToConstantHeading(new Pose2d(" +
                    Coords.get(index).getX()+

                    ","+
                    Coords.get(index).getY()+
                    " ),"+ heading+ ")   ");/**/
            index++;
        }
        //path.build();
        //System.out.println("");
        return path.build();

    }

}
