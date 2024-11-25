package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class AutoDriveSubsystem extends SubsystemBase {
    private Follower follower;
    private Telemetry telemetry;

    public AutoDriveSubsystem(Follower follower, Telemetry telemetry, Pose startPose){
        this.follower = follower;
        this.telemetry = telemetry;
        setStartingPose(startPose);
    }

    public void followPath(Path path, boolean holdEnd){
        follower.followPath(path, holdEnd);
    }
    public void followPath(PathChain pathChain, boolean holdEnd){
        follower.followPath(pathChain, holdEnd);
    }
    public void setMaxPower(double maxPower){
        follower.setMaxPower(maxPower);
    }
    public void setPose(Pose pose){
        follower.setPose(pose);
    }
    public void setStartingPose(Pose pose){
        follower.setStartingPose(pose);
    }
    public void holdPoint(BezierPoint point, double heading){
        follower.holdPoint(point, heading);
    }
    public void update(){
        follower.update();
    }
    public boolean isBusy(){
        return follower.isBusy();
    }
    public void breakFollowing(){
        follower.breakFollowing();
    }
    public PathBuilder pathBuilder(){
        return follower.pathBuilder();
    }
    public void telemetryDebug(Telemetry telemetry){
        follower.telemetryDebug(telemetry);
    }
    public Path getCurrentPath(){
        return follower.getCurrentPath();
    }
    public boolean atParametricEnd(){
        return follower.atParametricEnd();
    }
    public void setCurrentPoseWithOffset(Pose pose){
        follower.setCurrentPoseWithOffset(pose);
    }
    public void setXOffset(double xOffset){
        follower.setXOffset(xOffset);
    }
    public void setYOffset(double yOffset){
        follower.setYOffset(yOffset);
    }
    public void setHeadingOffset(double headingOffset){
        follower.setHeadingOffset(headingOffset);
    }
    public Pose getPose(){
        return follower.getPose();
    }
    public void resetOffset(){
        follower.resetOffset();
    }
    public void startTeleopDrive(){
        follower.startTeleopDrive();
    }
    public void setTeleOpMovementVectors(double forwardSpeed, double strafeSpeed, double heading, boolean robotCentric){
        follower.setTeleOpMovementVectors(forwardSpeed, strafeSpeed, heading);
    }
    public void holdPosition(){
        follower.holdPoint(new BezierPoint(new Point(getPose())), getPose().getHeading());
    }

}
