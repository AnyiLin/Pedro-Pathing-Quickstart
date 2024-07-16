package org.firstinspires.ftc.teamcode.config.pedroPathing.follower;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.config.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.PathCallback;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.config.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.PIDFController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * This is the Follower class. It handles the actual following of the paths and all the on-the-fly
 * calculations that are relevant for movement.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/4/2024
 */
@Config
public class Follower {
    private HardwareMap hardwareMap;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    private DriveVectorScaler driveVectorScaler;

    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;

    private Pose closestPose;

    private Path currentPath;

    private PathChain currentPathChain;

    private final int BEZIER_CURVE_BINARY_STEP_LIMIT = FollowerConstants.BEZIER_CURVE_BINARY_STEP_LIMIT;
    private final int AVERAGED_VELOCITY_SAMPLE_NUMBER = FollowerConstants.AVERAGED_VELOCITY_SAMPLE_NUMBER;

    private int chainIndex;

    private long[] pathStartTimes;

    private boolean followingPathChain;
    private boolean holdingPosition;
    private boolean isBusy;
    private boolean auto = true;
    private boolean reachedParametricPathEnd;
    private boolean holdPositionAtEnd;

    private double maxPower = 1;
    private double previousSmallTranslationalIntegral;
    private double previousLargeTranslationalIntegral;
    private double holdPointTranslationalScaling = FollowerConstants.holdPointTranslationalScaling;
    private double holdPointHeadingScaling = FollowerConstants.holdPointHeadingScaling;
    public double driveError;
    public double headingError;

    private long reachedParametricPathEndTime;

    private double[] drivePowers;

    private Vector[] teleOpMovementVectors = new Vector[]{new Vector(), new Vector(), new Vector()};

    private ArrayList<Vector> velocities = new ArrayList<>();
    private ArrayList<Vector> accelerations = new ArrayList<>();

    private Vector averageVelocity;
    private Vector averagePreviousVelocity;
    private Vector averageAcceleration;
    private Vector smallTranslationalIntegralVector;
    private Vector largeTranslationalIntegralVector;
    public Vector driveVector;
    public Vector headingVector;
    public Vector translationalVector;
    public Vector centripetalVector;
    public Vector correctiveVector;

    private PIDFController smallTranslationalPIDF = new PIDFController(FollowerConstants.smallTranslationalPIDFCoefficients);
    private PIDFController smallTranslationalIntegral = new PIDFController(FollowerConstants.smallTranslationalIntegral);
    private PIDFController largeTranslationalPIDF = new PIDFController(FollowerConstants.largeTranslationalPIDFCoefficients);
    private PIDFController largeTranslationalIntegral = new PIDFController(FollowerConstants.largeTranslationalIntegral);
    private PIDFController smallHeadingPIDF = new PIDFController(FollowerConstants.smallHeadingPIDFCoefficients);
    private PIDFController largeHeadingPIDF = new PIDFController(FollowerConstants.largeHeadingPIDFCoefficients);
    private PIDFController smallDrivePIDF = new PIDFController(FollowerConstants.smallDrivePIDFCoefficients);
    private PIDFController largeDrivePIDF = new PIDFController(FollowerConstants.largeDrivePIDFCoefficients);


    public static boolean drawOnDashboard = true;
    public static boolean useTranslational = true;
    public static boolean useCentripetal = true;
    public static boolean useHeading = true;
    public static boolean useDrive = true;

    /**
     * This creates a new Follower given a HardwareMap.
     *
     * @param hardwareMap HardwareMap required
     */
    public Follower(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initialize();
    }

    /**
     * This creates a new Follower given a HardwareMap and sets whether the Follower is being used
     * in autonomous or teleop.
     *
     * @param hardwareMap HardwareMap required
     * @param setAuto     sets whether or not the Follower is being used in autonomous or teleop
     */
    public Follower(HardwareMap hardwareMap, boolean setAuto) {
        this.hardwareMap = hardwareMap;
        setAuto(setAuto);
        initialize();
    }

    /**
     * This initializes the follower.
     * In this, the DriveVectorScaler and PoseUpdater is instantiated, the drive motors are
     * initialized and their behavior is set, and the variables involved in approximating first and
     * second derivatives for teleop are set.
     */
    public void initialize() {
        driveVectorScaler = new DriveVectorScaler(FollowerConstants.frontLeftVector);
        poseUpdater = new PoseUpdater(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        // TODO: Make sure that this is the direction your motors need to be reversed in.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        for (int i = 0; i < AVERAGED_VELOCITY_SAMPLE_NUMBER; i++) {
            velocities.add(new Vector());
        }
        for (int i = 0; i < AVERAGED_VELOCITY_SAMPLE_NUMBER / 2; i++) {
            accelerations.add(new Vector());
        }
        calculateAveragedVelocityAndAcceleration();

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);
    }

    /**
     * This sets the maximum power the motors are allowed to use.
     *
     * @param set This caps the motor power from [0, 1].
     */
    public void setMaxPower(double set) {
        maxPower = MathFunctions.clamp(set, 0, 1);
    }

    /**
     * This handles the limiting of the drive powers array to the max power.
     */
    public void limitDrivePowers() {
        for (int i = 0; i < drivePowers.length; i++) {
            if (Math.abs(drivePowers[i]) > maxPower) {
                drivePowers[i] = maxPower * MathFunctions.getSign(drivePowers[i]);
            }
        }
    }

    /**
     * This gets a Point from the current Path from a specified t-value.
     *
     * @return returns the Point.
     */
    public Point getPointFromPath(double t) {
        if (currentPath != null) {
            return currentPath.getPoint(t);
        } else {
            return null;
        }
    }

    /**
     * This returns the current pose from the PoseUpdater.
     *
     * @return returns the pose
     */
    public Pose getPose() {
        return poseUpdater.getPose();
    }

    /**
     * This sets the current pose in the PoseUpdater without using offsets.
     *
     * @param pose The pose to set the current pose to.
     */
    public void setPose(Pose pose) {
        poseUpdater.setPose(pose);
    }

    /**
     * This returns the current velocity of the robot as a Vector.
     *
     * @return returns the current velocity as a Vector.
     */
    public Vector getVelocity() {
        return poseUpdater.getVelocity();
    }

    /**
     * This returns the current acceleration of the robot as a Vector.
     *
     * @return returns the current acceleration as a Vector.
     */
    public Vector getAcceleration() {
        return poseUpdater.getAcceleration();
    }

    /**
     * This returns the magnitude of the current velocity. For when you only need the magnitude.
     *
     * @return returns the magnitude of the current velocity.
     */
    public double getVelocityMagnitude() {
        return poseUpdater.getVelocity().getMagnitude();
    }

    /**
     * This sets the starting pose. Do not run this after moving at all.
     *
     * @param pose the pose to set the starting pose to.
     */
    public void setStartingPose(Pose pose) {
        poseUpdater.setStartingPose(pose);
    }

    /**
     * This sets the current pose, using offsets so no reset time delay. This is better than the
     * Road Runner reset, in general. Think of using offsets as setting trim in an aircraft. This can
     * be reset as well, so beware of using the resetOffset() method.
     *
     * @param set The pose to set the current pose to.
     */
    public void setCurrentPoseWithOffset(Pose set) {
        poseUpdater.setCurrentPoseWithOffset(set);
    }

    /**
     * This sets the offset for only the x position.
     *
     * @param xOffset This sets the offset.
     */
    public void setXOffset(double xOffset) {
        poseUpdater.setXOffset(xOffset);
    }

    /**
     * This sets the offset for only the y position.
     *
     * @param yOffset This sets the offset.
     */
    public void setYOffset(double yOffset) {
        poseUpdater.setYOffset(yOffset);
    }

    /**
     * This sets the offset for only the heading.
     *
     * @param headingOffset This sets the offset.
     */
    public void setHeadingOffset(double headingOffset) {
        poseUpdater.setHeadingOffset(headingOffset);
    }

    /**
     * This returns the x offset.
     *
     * @return returns the x offset.
     */
    public double getXOffset() {
        return poseUpdater.getXOffset();
    }

    /**
     * This returns the y offset.
     *
     * @return returns the y offset.
     */
    public double getYOffset() {
        return poseUpdater.getYOffset();
    }

    /**
     * This returns the heading offset.
     *
     * @return returns the heading offset.
     */
    public double getHeadingOffset() {
        return poseUpdater.getHeadingOffset();
    }

    /**
     * This resets all offsets set to the PoseUpdater. If you have reset your pose using the
     * setCurrentPoseUsingOffset(Pose set) method, then your pose will be returned to what the
     * PoseUpdater thinks your pose would be, not the pose you reset to.
     */
    public void resetOffset() {
        poseUpdater.resetOffset();
    }

    /**
     * This holds a Point.
     *
     * @param point   the Point to stay at.
     * @param heading the heading to face.
     */
    public void holdPoint(BezierPoint point, double heading) {
        breakFollowing();
        holdingPosition = true;
        isBusy = false;
        followingPathChain = false;
        currentPath = new Path(point);
        currentPath.setConstantHeadingInterpolation(heading);
        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), 1);
    }

    /**
     * This follows a Path.
     * This also makes the Follower hold the last Point on the Path.
     *
     * @param path the Path to follow.
     */
    public void followPath(Path path, boolean holdEnd) {
        breakFollowing();
        holdPositionAtEnd = holdEnd;
        isBusy = true;
        followingPathChain = false;
        currentPath = path;
        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
    }

    /**
     * This follows a Path.
     *
     * @param path the Path to follow.
     */
    public void followPath(Path path) {
        followPath(path, false);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     * This also makes the Follower hold the last Point on the PathChain.
     *
     * @param pathChain the PathChain to follow.
     */
    public void followPath(PathChain pathChain, boolean holdEnd) {
        breakFollowing();
        holdPositionAtEnd = holdEnd;
        pathStartTimes = new long[pathChain.size()];
        pathStartTimes[0] = System.currentTimeMillis();
        isBusy = true;
        followingPathChain = true;
        chainIndex = 0;
        currentPathChain = pathChain;
        currentPath = pathChain.getPath(chainIndex);
        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
    }

    /**
     * This follows a PathChain. Drive vector projection is only done on the last Path.
     *
     * @param pathChain the PathChain to follow.
     */
    public void followPath(PathChain pathChain) {
        followPath(pathChain, false);
    }

    /**
     * This calls an update to the PoseUpdater, which updates the robot's current position estimate.
     * This also updates all the Follower's PIDFs, which updates the motor powers.
     */
    public void update() {
        poseUpdater.update();

        if (drawOnDashboard) {
            dashboardPoseTracker.update();
        }

        if (auto) {
            if (holdingPosition) {
                closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), 1);

                drivePowers = driveVectorScaler.getDrivePowers(MathFunctions.scalarMultiplyVector(getTranslationalCorrection(), holdPointTranslationalScaling), MathFunctions.scalarMultiplyVector(getHeadingVector(), holdPointHeadingScaling), new Vector(), poseUpdater.getPose().getHeading());

                limitDrivePowers();

                for (int i = 0; i < motors.size(); i++) {
                    motors.get(i).setPower(drivePowers[i]);
                }
            } else {
                if (isBusy) {
                    closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);

                    if (followingPathChain) updateCallbacks();

                    drivePowers = driveVectorScaler.getDrivePowers(getCorrectiveVector(), getHeadingVector(), getDriveVector(), poseUpdater.getPose().getHeading());

                    limitDrivePowers();

                    for (int i = 0; i < motors.size(); i++) {
                        motors.get(i).setPower(drivePowers[i]);
                    }
                }
                if (currentPath.isAtParametricEnd()) {
                    if (followingPathChain && chainIndex < currentPathChain.size() - 1) {
                        // Not at last path, keep going
                        breakFollowing();
                        pathStartTimes[chainIndex] = System.currentTimeMillis();
                        isBusy = true;
                        followingPathChain = true;
                        chainIndex++;
                        currentPath = currentPathChain.getPath(chainIndex);
                        closestPose = currentPath.getClosestPoint(poseUpdater.getPose(), BEZIER_CURVE_BINARY_STEP_LIMIT);
                    } else {
                        // At last path, run some end detection stuff
                        // set isBusy to false if at end
                        if (!reachedParametricPathEnd) {
                            reachedParametricPathEnd = true;
                            reachedParametricPathEndTime = System.currentTimeMillis();
                        }

                        if ((System.currentTimeMillis() - reachedParametricPathEndTime > currentPath.getPathEndTimeoutConstraint()) || (poseUpdater.getVelocity().getMagnitude() < currentPath.getPathEndVelocityConstraint() && MathFunctions.distance(poseUpdater.getPose(), closestPose) < currentPath.getPathEndTranslationalConstraint() && MathFunctions.getSmallestAngleDifference(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()) < currentPath.getPathEndHeadingConstraint())) {
                            if (holdPositionAtEnd) {
                                holdPositionAtEnd = false;
                                holdPoint(new BezierPoint(currentPath.getLastControlPoint()), currentPath.getHeadingGoal(1));
                            } else {
                                breakFollowing();
                            }
                        }
                    }
                }
            }
        } else {
            velocities.add(poseUpdater.getVelocity());
            velocities.remove(velocities.get(velocities.size() - 1));

            calculateAveragedVelocityAndAcceleration();

            drivePowers = driveVectorScaler.getDrivePowers(teleOpMovementVectors[0], teleOpMovementVectors[1], teleOpMovementVectors[2], poseUpdater.getPose().getHeading());

            limitDrivePowers();

            for (int i = 0; i < motors.size(); i++) {
                motors.get(i).setPower(drivePowers[i]);
            }
        }
    }

    /**
     * This calculates an averaged approximate velocity and acceleration. This is used for a
     * real-time correction of centripetal force, which is used in teleop.
     */
    public void calculateAveragedVelocityAndAcceleration() {
        averageVelocity = new Vector();
        averagePreviousVelocity = new Vector();

        for (int i = 0; i < velocities.size() / 2; i++) {
            averageVelocity = MathFunctions.addVectors(averageVelocity, velocities.get(i));
        }
        averageVelocity = MathFunctions.scalarMultiplyVector(averageVelocity, 1.0 / ((double) velocities.size() / 2));

        for (int i = velocities.size() / 2; i < velocities.size(); i++) {
            averagePreviousVelocity = MathFunctions.addVectors(averagePreviousVelocity, velocities.get(i));
        }
        averagePreviousVelocity = MathFunctions.scalarMultiplyVector(averagePreviousVelocity, 1.0 / ((double) velocities.size() / 2));

        accelerations.add(MathFunctions.subtractVectors(averageVelocity, averagePreviousVelocity));
        accelerations.remove(accelerations.size() - 1);

        averageAcceleration = new Vector();

        for (int i = 0; i < accelerations.size(); i++) {
            averageAcceleration = MathFunctions.addVectors(averageAcceleration, accelerations.get(i));
        }
        averageAcceleration = MathFunctions.scalarMultiplyVector(averageAcceleration, 1.0 / accelerations.size());
    }

    /**
     * This checks if any PathCallbacks should be run right now, and runs them if applicable.
     */
    public void updateCallbacks() {
        for (PathCallback callback : currentPathChain.getCallbacks()) {
            if (!callback.hasBeenRun()) {
                if (callback.getType() == PathCallback.PARAMETRIC) {
                    // parametric call back
                    if (chainIndex == callback.getIndex() && (getCurrentTValue() >= callback.getStartCondition() || MathFunctions.roughlyEquals(getCurrentTValue(), callback.getStartCondition()))) {
                        callback.run();
                    }
                } else {
                    // time based call back
                    if (chainIndex >= callback.getIndex() && System.currentTimeMillis() - pathStartTimes[callback.getIndex()] > callback.getStartCondition()) {
                        callback.run();
                    }

                }
            }
        }
    }

    /**
     * This resets the PIDFs and stops following the current Path.
     */
    public void breakFollowing() {
        holdingPosition = false;
        isBusy = false;
        reachedParametricPathEnd = false;
        smallDrivePIDF.reset();
        largeDrivePIDF.reset();
        smallHeadingPIDF.reset();
        largeHeadingPIDF.reset();
        smallTranslationalPIDF.reset();
        smallTranslationalIntegral.reset();
        smallTranslationalIntegralVector = new Vector();
        previousSmallTranslationalIntegral = 0;
        largeTranslationalPIDF.reset();
        largeTranslationalIntegral.reset();
        largeTranslationalIntegralVector = new Vector();
        previousLargeTranslationalIntegral = 0;
        driveVector = new Vector();
        headingVector = new Vector();
        translationalVector = new Vector();
        centripetalVector = new Vector();
        correctiveVector = new Vector();
        driveError = 0;
        headingError = 0;

        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setPower(0);
        }
    }

    /**
     * This returns if the Follower is currently following a Path or a PathChain.
     *
     * @return returns if the Follower is busy.
     */
    public boolean isBusy() {
        return isBusy;
    }

    /**
     * Sets the correctional, heading, and drive movement vectors for teleop enhancements.
     * The correctional Vector only accounts for an approximated centripetal correction.
     */
    public void setMovementVectors(Vector correctional, Vector heading, Vector drive) {
        teleOpMovementVectors = new Vector[]{correctional, heading, drive};
    }

    /**
     * This returns a Vector in the direction the robot must go to move along the path. This Vector
     * takes into account the projected position of the robot to calculate how much power is needed.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the drive vector.
     */
    public Vector getDriveVector() {
        if (!useDrive) return new Vector();
        if (followingPathChain && chainIndex < currentPathChain.size() - 1) {
            return new Vector(1, currentPath.getClosestPointTangentVector().getTheta());
        }

        driveError = getDriveVelocityError();

        if (Math.abs(driveError) < FollowerConstants.drivePIDFSwitch) {
            smallDrivePIDF.updateError(driveError);
            driveVector = new Vector(MathFunctions.clamp(smallDrivePIDF.runPIDF() + FollowerConstants.smallDrivePIDFFeedForward * MathFunctions.getSign(driveError), -1, 1), currentPath.getClosestPointTangentVector().getTheta());
            return MathFunctions.copyVector(driveVector);
        }

        largeDrivePIDF.updateError(driveError);
        driveVector = new Vector(MathFunctions.clamp(largeDrivePIDF.runPIDF() + FollowerConstants.largeDrivePIDFFeedForward * MathFunctions.getSign(driveError), -1, 1), currentPath.getClosestPointTangentVector().getTheta());
        return MathFunctions.copyVector(driveVector);
    }

    /**
     * This returns the velocity the robot needs to be at to make it to the end of the Path
     * at some specified deceleration (well technically just some negative acceleration).
     *
     * @return returns the projected velocity.
     */
    public double getDriveVelocityError() {
        double distanceToGoal;
        if (!currentPath.isAtParametricEnd()) {
            distanceToGoal = currentPath.length() * (1 - currentPath.getClosestPointTValue());
        } else {
            Vector offset = new Vector();
            offset.setOrthogonalComponents(getPose().getX() - currentPath.getLastControlPoint().getX(), getPose().getY() - currentPath.getLastControlPoint().getY());
            distanceToGoal = MathFunctions.dotProduct(currentPath.getEndTangent(), offset);
        }

        Vector distanceToGoalVector = MathFunctions.scalarMultiplyVector(MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector()), distanceToGoal);
        Vector velocity = new Vector(MathFunctions.dotProduct(getVelocity(), MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta());

        Vector forwardHeadingVector = new Vector(1.0, poseUpdater.getPose().getHeading());
        double forwardVelocity = MathFunctions.dotProduct(forwardHeadingVector, velocity);
        double forwardDistanceToGoal = MathFunctions.dotProduct(forwardHeadingVector, distanceToGoalVector);
        double forwardVelocityGoal = MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(-2 * currentPath.getZeroPowerAccelerationMultiplier() * FollowerConstants.forwardZeroPowerAcceleration * forwardDistanceToGoal));
        double forwardVelocityZeroPowerDecay = forwardVelocity - MathFunctions.getSign(forwardDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(forwardVelocity, 2) + 2 * FollowerConstants.forwardZeroPowerAcceleration * forwardDistanceToGoal));

        Vector lateralHeadingVector = new Vector(1.0, poseUpdater.getPose().getHeading() - Math.PI / 2);
        double lateralVelocity = MathFunctions.dotProduct(lateralHeadingVector, velocity);
        double lateralDistanceToGoal = MathFunctions.dotProduct(lateralHeadingVector, distanceToGoalVector);
        double lateralVelocityGoal = MathFunctions.getSign(lateralDistanceToGoal) * Math.sqrt(Math.abs(-2 * currentPath.getZeroPowerAccelerationMultiplier() * FollowerConstants.lateralZeroPowerAcceleration * lateralDistanceToGoal));
        double lateralVelocityZeroPowerDecay = lateralVelocity - MathFunctions.getSign(lateralDistanceToGoal) * Math.sqrt(Math.abs(Math.pow(lateralVelocity, 2) + 2 * FollowerConstants.lateralZeroPowerAcceleration * lateralDistanceToGoal));

        Vector forwardVelocityError = new Vector(forwardVelocityGoal - forwardVelocityZeroPowerDecay - forwardVelocity, forwardHeadingVector.getTheta());
        Vector lateralVelocityError = new Vector(lateralVelocityGoal - lateralVelocityZeroPowerDecay - lateralVelocity, lateralHeadingVector.getTheta());
        Vector velocityErrorVector = MathFunctions.addVectors(forwardVelocityError, lateralVelocityError);

        return velocityErrorVector.getMagnitude() * MathFunctions.getSign(MathFunctions.dotProduct(velocityErrorVector, currentPath.getClosestPointTangentVector()));
    }

    /**
     * This returns a Vector in the direction of the robot that contains the heading correction
     * as its magnitude. Positive heading correction turns the robot counter-clockwise, and negative
     * heading correction values turn the robot clockwise. So basically, Pedro Pathing uses a right-
     * handed coordinate system.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the heading vector.
     */
    public Vector getHeadingVector() {
        if (!useHeading) return new Vector();
        headingError = MathFunctions.getTurnDirection(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()) * MathFunctions.getSmallestAngleDifference(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal());
        if (Math.abs(headingError) < FollowerConstants.headingPIDFSwitch) {
            smallHeadingPIDF.updateError(headingError);
            headingVector = new Vector(MathFunctions.clamp(smallHeadingPIDF.runPIDF() + FollowerConstants.smallHeadingPIDFFeedForward * MathFunctions.getTurnDirection(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()), -1, 1), poseUpdater.getPose().getHeading());
            return MathFunctions.copyVector(headingVector);
        }
        largeHeadingPIDF.updateError(headingError);
        headingVector = new Vector(MathFunctions.clamp(largeHeadingPIDF.runPIDF() + FollowerConstants.largeHeadingPIDFFeedForward * MathFunctions.getTurnDirection(poseUpdater.getPose().getHeading(), currentPath.getClosestPointHeadingGoal()), -1, 1), poseUpdater.getPose().getHeading());
        return MathFunctions.copyVector(headingVector);
    }

    /**
     * This returns a combined Vector in the direction the robot must go to correct both translational
     * error as well as centripetal force.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the corrective vector.
     */
    public Vector getCorrectiveVector() {
        Vector centripetal = getCentripetalForceCorrection();
        Vector translational = getTranslationalCorrection();
        Vector corrective = MathFunctions.addVectors(centripetal, translational);

        if (corrective.getMagnitude() > 1) {
            return MathFunctions.addVectors(centripetal, MathFunctions.scalarMultiplyVector(translational, driveVectorScaler.findNormalizingScaling(centripetal, translational)));
        }

        correctiveVector = MathFunctions.copyVector(corrective);

        return corrective;
    }

    /**
     * This returns a Vector in the direction the robot must go to account for only translational
     * error.
     * <p>
     * Note: This vector is clamped to be at most 1 in magnitude.
     *
     * @return returns the translational correction vector.
     */
    public Vector getTranslationalCorrection() {
        if (!useTranslational) return new Vector();
        Vector translationalVector = new Vector();
        double x = closestPose.getX() - poseUpdater.getPose().getX();
        double y = closestPose.getY() - poseUpdater.getPose().getY();
        translationalVector.setOrthogonalComponents(x, y);

        if (!(currentPath.isAtParametricEnd() || currentPath.isAtParametricStart())) {
            translationalVector = MathFunctions.subtractVectors(translationalVector, new Vector(MathFunctions.dotProduct(translationalVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));

            smallTranslationalIntegralVector = MathFunctions.subtractVectors(smallTranslationalIntegralVector, new Vector(MathFunctions.dotProduct(smallTranslationalIntegralVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));
            largeTranslationalIntegralVector = MathFunctions.subtractVectors(largeTranslationalIntegralVector, new Vector(MathFunctions.dotProduct(largeTranslationalIntegralVector, MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), currentPath.getClosestPointTangentVector().getTheta()));
        }

        if (MathFunctions.distance(poseUpdater.getPose(), closestPose) < FollowerConstants.translationalPIDFSwitch) {
            smallTranslationalIntegral.updateError(translationalVector.getMagnitude());
            smallTranslationalIntegralVector = MathFunctions.addVectors(smallTranslationalIntegralVector, new Vector(smallTranslationalIntegral.runPIDF() - previousSmallTranslationalIntegral, translationalVector.getTheta()));
            previousSmallTranslationalIntegral = smallTranslationalIntegral.runPIDF();

            smallTranslationalPIDF.updateError(translationalVector.getMagnitude());
            translationalVector.setMagnitude(smallTranslationalPIDF.runPIDF() + FollowerConstants.smallTranslationalPIDFFeedForward);
            translationalVector = MathFunctions.addVectors(translationalVector, smallTranslationalIntegralVector);
        } else {
            largeTranslationalIntegral.updateError(translationalVector.getMagnitude());
            largeTranslationalIntegralVector = MathFunctions.addVectors(largeTranslationalIntegralVector, new Vector(largeTranslationalIntegral.runPIDF() - previousLargeTranslationalIntegral, translationalVector.getTheta()));
            previousLargeTranslationalIntegral = largeTranslationalIntegral.runPIDF();

            largeTranslationalPIDF.updateError(translationalVector.getMagnitude());
            translationalVector.setMagnitude(largeTranslationalPIDF.runPIDF() + FollowerConstants.largeTranslationalPIDFFeedForward);
            translationalVector = MathFunctions.addVectors(translationalVector, largeTranslationalIntegralVector);
        }

        translationalVector.setMagnitude(MathFunctions.clamp(translationalVector.getMagnitude(), 0, 1));

        this.translationalVector = MathFunctions.copyVector(translationalVector);

        return translationalVector;
    }

    /**
     * This returns the raw translational error, or how far off the closest point the robot is.
     *
     * @return This returns the raw translational error as a Vector.
     */
    public Vector getTranslationalError() {
        Vector error = new Vector();
        double x = closestPose.getX() - poseUpdater.getPose().getX();
        double y = closestPose.getY() - poseUpdater.getPose().getY();
        error.setOrthogonalComponents(x, y);
        return error;
    }

    /**
     * This returns a Vector in the direction the robot must go to account for only centripetal
     * force.
     * <p>
     * Note: This vector is clamped to be between [0, 1] in magnitude.
     *
     * @return returns the centripetal force correction vector.
     */
    public Vector getCentripetalForceCorrection() {
        if (!useCentripetal) return new Vector();
        double curvature;
        if (auto) {
            curvature = currentPath.getClosestPointCurvature();
        } else {
            double yPrime = averageVelocity.getYComponent() / averageVelocity.getXComponent();
            double yDoublePrime = averageAcceleration.getYComponent() / averageVelocity.getXComponent();
            curvature = (yDoublePrime) / (Math.pow(Math.sqrt(1 + Math.pow(yPrime, 2)), 3));
        }
        if (Double.isNaN(curvature)) return new Vector();
        centripetalVector = new Vector(MathFunctions.clamp(FollowerConstants.centripetalScaling * FollowerConstants.mass * Math.pow(MathFunctions.dotProduct(poseUpdater.getVelocity(), MathFunctions.normalizeVector(currentPath.getClosestPointTangentVector())), 2) * curvature, -1, 1), currentPath.getClosestPointTangentVector().getTheta() + Math.PI / 2 * MathFunctions.getSign(currentPath.getClosestPointNormalVector().getTheta()));
        return centripetalVector;
    }

    /**
     * This returns the closest pose to the robot on the Path the Follower is currently following.
     * This closest pose is calculated through a binary search method with some specified number of
     * steps to search. By default, 10 steps are used, which should be more than enough.
     *
     * @return returns the closest pose.
     */
    public Pose getClosestPose() {
        return closestPose;
    }

    /**
     * This sets whether or not the Follower is being used in auto or teleop.
     *
     * @param set sets auto or not
     */
    public void setAuto(boolean set) {
        auto = set;
    }

    /**
     * This returns whether the follower is at the parametric end of its current Path.
     * The parametric end is determined by if the closest Point t-value is greater than some specified
     * end t-value.
     * If running a PathChain, this returns true only if at parametric end of last Path in the PathChain.
     *
     * @return returns whether the Follower is at the parametric end of its Path.
     */
    public boolean atParametricEnd() {
        if (followingPathChain) {
            if (chainIndex == currentPathChain.size() - 1) return currentPath.isAtParametricEnd();
            return false;
        }
        return currentPath.isAtParametricEnd();
    }

    /**
     * This returns the t value of the closest point on the current Path to the robot
     * In the absence of a current Path, it returns 1.0.
     *
     * @return returns the current t value.
     */
    public double getCurrentTValue() {
        if (isBusy) return currentPath.getClosestPointTValue();
        return 1.0;
    }

    /**
     * This returns the current path number. For following Paths, this will return 0. For PathChains,
     * this will return the current path number. For holding Points, this will also return 0.
     *
     * @return returns the current path number.
     */
    public double getCurrentPathNumber() {
        if (!followingPathChain) return 0;
        return chainIndex;
    }

    /**
     * This returns a new PathBuilder object for easily building PathChains.
     *
     * @return returns a new PathBuilder object.
     */
    public PathBuilder pathBuilder() {
        return new PathBuilder();
    }

    /**
     * This writes out information about the various motion Vectors to the Telemetry specified.
     *
     * @param telemetry this is an instance of Telemetry or the FTC Dashboard telemetry that this
     *                  method will use to output the debug data.
     */
    public void telemetryDebug(MultipleTelemetry telemetry) {
        telemetry.addData("follower busy", isBusy());
        telemetry.addData("heading error", headingError);
        telemetry.addData("heading vector magnitude", headingVector.getMagnitude());
        telemetry.addData("corrective vector magnitude", correctiveVector.getMagnitude());
        telemetry.addData("corrective vector heading", correctiveVector.getTheta());
        telemetry.addData("translational error magnitude", getTranslationalError().getMagnitude());
        telemetry.addData("translational error direction", getTranslationalError().getTheta());
        telemetry.addData("translational vector magnitude", translationalVector.getMagnitude());
        telemetry.addData("translational vector heading", translationalVector.getMagnitude());
        telemetry.addData("centripetal vector magnitude", centripetalVector.getMagnitude());
        telemetry.addData("centripetal vector heading", centripetalVector.getTheta());
        telemetry.addData("drive error", driveError);
        telemetry.addData("drive vector magnitude", driveVector.getMagnitude());
        telemetry.addData("drive vector heading", driveVector.getTheta());
        telemetry.addData("x", getPose().getX());
        telemetry.addData("y", getPose().getY());
        telemetry.addData("heading", getPose().getHeading());
        telemetry.addData("total heading", poseUpdater.getTotalHeading());
        telemetry.addData("velocity magnitude", getVelocity().getMagnitude());
        telemetry.addData("velocity heading", getVelocity().getTheta());
        telemetry.update();
        if (drawOnDashboard) {
            Drawing.drawDebug(this);
        }
    }

    /**
     * This writes out information about the various motion Vectors to the Telemetry specified.
     *
     * @param telemetry this is an instance of Telemetry or the FTC Dashboard telemetry that this
     *                  method will use to output the debug data.
     */
    public void telemetryDebug(Telemetry telemetry) {
        telemetryDebug(new MultipleTelemetry(telemetry));
    }

    /**
     * This returns the total number of radians the robot has turned.
     *
     * @return the total heading.
     */
    public double getTotalHeading() {
        return poseUpdater.getTotalHeading();
    }

    /**
     * This returns the current Path the Follower is following. This can be null.
     *
     * @return returns the current Path.
     */
    public Path getCurrentPath() {
        return currentPath;
    }

    /**
     * This returns the pose tracker for the robot to draw on the Dashboard.
     *
     * @return returns the pose tracker
     */
    public DashboardPoseTracker getDashboardPoseTracker() {
        return dashboardPoseTracker;
    }
}
