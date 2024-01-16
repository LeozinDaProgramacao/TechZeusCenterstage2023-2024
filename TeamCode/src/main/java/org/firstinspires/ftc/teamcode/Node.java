package com.mycompany.node;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;


/**
 *
 * @author carlo
 */
class Pair{
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

public class Node implements Comparable<Node> {
    // Id for readability of result purposes
    private static int idCounter = 0;
    public int id;

    // Parent in the path
    public Node parent = null;

    public List<Edge> neighbors;
    public static List<Node> allNodes= new ArrayList<>();

    // Evaluation functions
    public double f = Double.MAX_VALUE;
    public double g = Double.MAX_VALUE;
    // Hardcoded heuristic
    public double h;
    public double XPos;
    public double YPos;

    public static Node getClosestNode(Node curP) {
        double currdist = Double.MAX_VALUE;
        Node closest = null;
        for(Node n : allNodes){
            double dif = Math.sqrt((n.XPos-curP.XPos)*(n.XPos-curP.XPos)+(n.YPos-curP.YPos)*(n.YPos-curP.YPos));
            if (dif<currdist){
                currdist = dif;
                closest = n;
            }
        }
        return closest;
    }
    public static Node makebluegraph(){
        Node backdrop = new Node(48,36);
        Node behindBackdrop = new Node(36,36);
        Node center = new Node(30,0);
        Node frontbridgeL = new Node(12,12);
        Node frontbridgeR = new Node(12,-12);
        Node backbridgeL = new Node(-36,12);
        Node backbridgeR = new Node(-36,-12);
        Node pixel_stack_temp_goal = new Node(-52,0);


        backdrop.addBranch(behindBackdrop);

        behindBackdrop.addBranch(center);
        behindBackdrop.addBranch(frontbridgeL);

        center.addBranch(frontbridgeL);
        center.addBranch(frontbridgeR);

        frontbridgeL.addBranch(frontbridgeR);
        frontbridgeL.addBranch(backbridgeL);
        frontbridgeL.addBranch(backbridgeR);

        frontbridgeR.addBranch(backbridgeL);
        frontbridgeR.addBranch(backbridgeR);

        backbridgeL.addBranch(pixel_stack_temp_goal);
        backbridgeR.addBranch(pixel_stack_temp_goal);
        return pixel_stack_temp_goal;
    }
    /*public static void main(String[] args) {
        Node head = new Node(0,0);
        allNodes.add(head);
        Node n1 = new Node(0,6);
        allNodes.add(n1);
        Node n2 = new Node(6,0);
        allNodes.add(n2);
        Node target = new Node(2,4);
        allNodes.add(target);

        head.addBranch(n1);
        head.addBranch(n2);
        //head.addBranch(target);
        n2.addBranch(target);
        n1.addBranch(target);

        Node start = getClosestNode(new Node(3,4));
        Node res = aStar(start, target);

        printPath(res);
    }*/

    public Node(double XPos,double YPos){
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

    public void addBranch(Node node){
        Edge newEdge = new Edge(Math.sqrt((node.XPos-this.XPos)*(node.XPos-this.XPos)+(node.YPos-this.YPos)*(node.YPos-this.YPos)), node);
        neighbors.add(newEdge);
    }

    public double calculateHeuristic(Node target){
        return Math.sqrt((target.XPos-this.XPos)*(target.XPos-this.XPos)+(target.YPos-this.YPos)*(target.YPos-this.YPos));
    }
    public static Node aStar(Node start, Node target){
        PriorityQueue<Node> closedList = new PriorityQueue<>();
        PriorityQueue<Node> openList = new PriorityQueue<>();
        start.g=0;
        start.f = start.g + start.calculateHeuristic(target);
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
                    m.f = m.g + m.calculateHeuristic(target);
                    openList.add(m);
                } else {
                    if(totalWeight < m.g){
                        m.parent = n;
                        m.g = totalWeight;
                        m.f = m.g + m.calculateHeuristic(target);

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

    public static Trajectory //replace void with TrajectorySequence or sth like that
    printPath(Node target, SampleMecanumDrive drive, Pose2d currentpose){
        Node n = target;

        List<Pair> Coords = new ArrayList<Pair>();
        Pair Coord=null;
        List<Integer> ids = new ArrayList<>();

        while(n.parent != null){
            Coord = new Pair(n.XPos,n.YPos);
            Coords.add(Coord);
            ids.add(n.id);
            n = n.parent;
        }
        ids.add(n.id);
        Coord = new Pair(n.XPos,n.YPos);
        Coords.add(Coord);
        Collections.reverse(Coords);
        Collections.reverse(ids);
        int index=0;
        System.out.println(Coords);
        TrajectoryBuilder path = drive.trajectoryBuilder(currentpose);
        //TrajectorySequence path = new TrajectorySequenceBuilder(currentpose)
        for(int id : ids){

            path.lineTo(new Vector2d(Coords.get(index).getX(),Coords.get(index).getY()));


            /*System.out.print(id + " (" +
                    Coords.get(index).getX()+
                    ","+
                    Coords.get(index).getY()+
                    ")");*/
            index++;
        }
        return path.build();
        //path.build();
        //System.out.println("");
        //return path;
    }
}