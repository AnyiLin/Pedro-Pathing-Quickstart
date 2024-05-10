package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the RRToPedroThreeWheelLocalizer class. This class extends the Localizer superclass and
 * is intended to adapt the old Road Runner three wheel odometry localizer to the new Pedro Pathing
 * localizer system.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 5/9/2024
 */
public class RRToPedroThreeWheelLocalizer extends Localizer {
    private RoadRunnerThreeWheelLocalizer localizer;
    private double totalHeading;
    private Pose startPose;
    private Pose previousPose;

    public RRToPedroThreeWheelLocalizer(HardwareMap hardwareMap) {
        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        localizer = new RoadRunnerThreeWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        startPose = new Pose();
        previousPose = new Pose();
    }
    @Override
    public Pose getPose() {
        Pose2d pose = localizer.getPoseEstimate();
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    @Override
    public Pose getVelocity() {
        Pose2d pose = localizer.getPoseVelocity();
        return new Pose(pose.getX(), pose.getY(), pose.getHeading());
    }

    @Override
    public Vector getVelocityVector() {
        Pose2d pose = localizer.getPoseVelocity();
        Vector returnVector = new Vector();
        returnVector.setOrthogonalComponents(pose.getX(), pose.getY());
        return returnVector;
    }

    @Override
    public void setStartPose(Pose setStart) {
        Pose oldStart = startPose;
        startPose = setStart;
        Pose startDiff = MathFunctions.subtractPoses(startPose, oldStart);
        localizer.setPoseEstimate(new Pose2d(getPose().getX() + startDiff.getX(), getPose().getY() + startDiff.getY(), getPose().getHeading() + startDiff.getHeading()));
    }

    @Override
    public void setPose(Pose setPose) {
        localizer.setPoseEstimate(new Pose2d(setPose.getX(), setPose.getY(), setPose.getHeading()));
    }

    @Override
    public void update() {
        totalHeading += MathFunctions.getTurnDirection(previousPose.getHeading(), getPose().getHeading()) * MathFunctions.getSmallestAngleDifference(previousPose.getHeading(), getPose().getHeading());
        previousPose = getPose();
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public double getForwardMultiplier() {
        return RoadRunnerThreeWheelLocalizer.encoderTicksToInches(1) * RoadRunnerThreeWheelLocalizer.X_MULTIPLIER;
    }

    @Override
    public double getLateralMultiplier() {
        return RoadRunnerThreeWheelLocalizer.encoderTicksToInches(1) * RoadRunnerThreeWheelLocalizer.Y_MULTIPLIER;
    }

    @Override
    public double getTurningMultiplier() {
        return (getForwardMultiplier() + getLateralMultiplier()) / 2;
    }
}
