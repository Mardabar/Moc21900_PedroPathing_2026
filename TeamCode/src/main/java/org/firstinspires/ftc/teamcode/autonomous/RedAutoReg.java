package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "RedAutoReg", group = "autonomous")
public class RedAutoReg extends OpMode{

    // PEDROPATHING VARS

    private Follower fol;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // CAMERA VARS

    private Limelight3A cam;
    private int obNum = 21;

    // POSITIONS

    private final Pose startPose = new Pose(56, 8, Math.toRadians(0));

        // Obelisk #21
    private final Pose Ob21Grab1GP1 = new Pose(31, 35.5, Math.toRadians(0)); // POSITION
    private final Pose Ob21Grab1GP1CP = new Pose(56, 35.5, Math.toRadians(0)); // CONTROL POINT

        // Obelisk #22


        // Obelisk #23


    // PATH CHAINS

    private PathChain pathOb21Grab1GP1;
    private PathChain pathOb22Grab1G1;
    private PathChain pathOb23Grab1PP1;

    public void buildPaths(){
        if (obNum == 21){

        }
        else if (obNum == 22){

        }
        else{

        }
    }

    @Override
    public void init(){

    }

    public void loop(){

    }
}
