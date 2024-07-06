package org.firstinspires.ftc.teamcode.opmode.example;


import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.config.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;

import java.util.concurrent.TimeUnit;

/**
 * This is an example auto that showcases movement and control of three servos autonomously.
 * It is able to detect the team element using a huskylens and then use that information to go to the correct spike mark and backdrop position.
 * There are examples of different ways to build paths.
 * A custom action system have been created that can be based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 1.0, 7/5/2024
 */

@Autonomous(name = "Example Auto Blue", group = "Examples")
public class ExampleAuto_Blue extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState, actionState, clawState;
    private String navigation;
    public ClawSubsystem claw;
    private HuskyLens huskyLens;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Centerstage, this would be blue far side/red human player station.)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y. **/
    //Start Pose
    private Pose startPose = new Pose(8.5, 84, 0);
    //Spike mark locations
    private Pose LeftSpikeMark = new Pose(52, 104, Math.toRadians(270));
    private Pose MiddleSpikeMark = new Pose(59, 94.5, Math.toRadians(270));
    private Pose RightSpikeMark = new Pose(52, 82.75, Math.toRadians(270));
    //Backdrop zone locations
    private Pose LeftBackdrop = new Pose(44, 121.75, Math.toRadians(270));
    private Pose MiddleBackdrop = new Pose(49.5, 121.75, Math.toRadians(270));
    private Pose RightBackdrop = new Pose(58, 121.25, Math.toRadians(270));
    private Pose WhiteBackdrop = new Pose(40, 122.25, Math.toRadians(270));

    // Poses and Paths for Purple and Yellow
    private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;
    private Path scoreSpikeMark, initialScoreOnBackdrop, scoreSpikeMarkChosen;

    // White Stack Cycle Poses + Path Chains
    private Pose TopTruss = new Pose(28, 84, Math.toRadians(270));
    private Pose BottomTruss = new Pose(28, 36, Math.toRadians(270));
    private Pose Stack = new Pose(46, 11.5, Math.toRadians(270));
    private PathChain cycleStackTo, cycleStackBack, cycleStackToBezier;

    /** Generate Spike Mark and Backdrop Paths based off of the team element location **/
    public void setBackdropGoalPose() {
        switch (navigation) {
            default:
            case "left":
                spikeMarkGoalPose = new Pose(LeftSpikeMark.getX(), LeftSpikeMark.getY(), Math.toRadians(270));
                initialBackdropGoalPose = new Pose(LeftBackdrop.getX(), LeftBackdrop.getY(), Math.toRadians(270));
                firstCycleBackdropGoalPose = new Pose(WhiteBackdrop.getX(), WhiteBackdrop.getY(), Math.toRadians(270));
                scoreSpikeMarkChosen = new Path(new BezierCurve(new Point(startPose), new Point(8.5,80.5,Point.CARTESIAN), new Point(48,135,Point.CARTESIAN), new Point(LeftSpikeMark)));
                break;
            case "middle":
                spikeMarkGoalPose = new Pose(MiddleSpikeMark.getX(), MiddleSpikeMark.getY()+3, Math.toRadians(270));
                initialBackdropGoalPose = new Pose(MiddleBackdrop.getX(), MiddleBackdrop.getY(),Math.toRadians(270));
                firstCycleBackdropGoalPose = new Pose(MiddleBackdrop.getX(), MiddleBackdrop.getY(), Math.toRadians(270));
                scoreSpikeMarkChosen = new Path(new BezierCurve(new Point(startPose), new Point(8.5,12+72+5,Point.CARTESIAN), new Point(-30+72+15,22+67.5+20+5,Point.CARTESIAN), new Point(MiddleSpikeMark)));
                break;
            case "right":
                spikeMarkGoalPose = new Pose(RightSpikeMark.getX(), RightSpikeMark.getY(), Math.toRadians(270));
                initialBackdropGoalPose = new Pose(RightBackdrop.getX(), RightBackdrop.getY(), Math.toRadians(270));
                firstCycleBackdropGoalPose = new Pose(RightBackdrop.getX(), RightBackdrop.getY(), Math.toRadians(270));
                scoreSpikeMarkChosen = new Path(new BezierCurve(new Point(startPose), new Point(8.5,12+72+5,Point.CARTESIAN), new Point(-36+72+16,80+17+4,Point.CARTESIAN), new Point(RightSpikeMark)));
                break;
        }
    }

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scoreSpikeMark = scoreSpikeMarkChosen;
        scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading(), spikeMarkGoalPose.getHeading());
        scoreSpikeMark.setPathEndTimeoutConstraint(0);

        /** There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require > 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://www.desmos.com/calculator/3so1zx0hcd).
         *    * BezierLines are straight, and require 2 points. There are the start and end points. **/

        initialScoreOnBackdrop = new Path(new BezierLine(new Point(spikeMarkGoalPose), new Point(initialBackdropGoalPose)));
        initialScoreOnBackdrop.setLinearHeadingInterpolation(spikeMarkGoalPose.getHeading(), initialBackdropGoalPose.getHeading());
        initialScoreOnBackdrop.setPathEndTimeoutConstraint(0);

        /** This is a path chain, defined on line 66
         * It, well, chains multiple paths together. Here we use a constant heading from the board to the stack.
         * On line 97, we set the Linear Interpolation,
         * which means that Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path */

        cycleStackTo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(initialBackdropGoalPose), new Point(TopTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierLine(new Point(TopTruss), new Point(BottomTruss)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .addPath(new BezierCurve(new Point(BottomTruss), new Point(12+13+1, 12, Point.CARTESIAN), new Point(31+12+1,36,Point.CARTESIAN), new Point(Stack)))
                .setConstantHeadingInterpolation(firstCycleBackdropGoalPose.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackBack = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Stack), new Point(BottomTruss)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(BottomTruss), new Point(TopTruss)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .addPath(new BezierLine(new Point(TopTruss), new Point(WhiteBackdrop)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();

        cycleStackToBezier = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(initialBackdropGoalPose), new Point(30+14,91.6, Point.CARTESIAN), new Point(13+14, 130.8, Point.CARTESIAN), new Point(BottomTruss)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .addPath(new BezierCurve(new Point(BottomTruss), new Point(20.5+14,10, Point.CARTESIAN), new Point(42+14,35, Point.CARTESIAN), new Point(Stack)))
                .setConstantHeadingInterpolation(WhiteBackdrop.getHeading())
                .setPathEndTimeoutConstraint(0)
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() function on line 193)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. **/
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                follower.followPath(scoreSpikeMark);
                setPathState(11);
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 2.6) {
                    claw.openLClaw();
                    setPathState(12);
                }
                break;
            case 12:
                if(follower.getPose().getY() > 120) {
                    setActionState(1);
                    follower.followPath(initialScoreOnBackdrop);
                    setPathState(13);
                }
                break;
        }
    }

    /** This switch is called continuously and runs the necessary actions, when finished, it will set the state to -1.
     * (Therefore, it will not run the action continuously) **/
    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                setClawState(0);
                setActionState(-1);
                break;
            case 1:
                setClawState(1);
                setActionState(-1);
                break;
        }
    }

    /** This switch is called continuously and runs the claw actions, when finished, it will set the state to -1.
     * (Therefore, it will not run the action continuously) **/
    public void clawUpdate() {
        switch (clawState) {
            case 0:
                claw.groundClaw();
                claw.closeClaws();
                setClawState(-1);
                break;
            case 1:
                claw.scoringClaw();
                setClawState(-1);
                break;
        }
    }


    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    public void setActionState(int aState) {
        actionState = aState;
        pathTimer.resetTimer();
        autonomousActionUpdate();
    }

    public void setClawState(int cState) {
        clawState = cState;
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the actions and movement of the robot
        follower.update();
        autonomousPathUpdate();
        autonomousActionUpdate();
        clawUpdate();

        //Huskylens Setup
        Deadline rateLimit = new Deadline(1, TimeUnit.SECONDS);
        rateLimit.expire();
        if (!huskyLens.knock()) { telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName()); }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        claw = new ClawSubsystem(hardwareMap);

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        claw.closeClaws();
        claw.startClaw();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

        // Scanning for Team Element
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (int i = 0; i < blocks.length; i++) {
            //----------------------------1----------------------------\\
            if (blocks[i].x <= 100 && blocks[i].id == 2) {
                navigation = "left";
            }
            if (blocks[i].x > 100 && blocks[i].x <= 270 && blocks[i].id == 2) {
                navigation = "middle";
            }
            if (blocks[i].x > 270 && blocks[i].id == 2) {
                navigation = "right";
            }
        }

        // After 4 Seconds, Robot Initialization is complete
        if (opmodeTimer.getElapsedTimeSeconds() > 4) {
            telemetry.addData("Init", "Finished");
        }
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        claw.closeClaws();
        setBackdropGoalPose();
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(10);
        setActionState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}