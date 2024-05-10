package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

/**
 * This is the Pose class. It defines poses in 2D space, like the Pose2D class in Road Runner except
 * in the Pedro Pathing code so I don't have to import the Road Runner library. A Pose consists of
 * two coordinates defining a position and a third value for the heading, so basically just defining
 * any position and orientation the robot can be at, unless your robot can fly for whatever reason.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */
public class Pose {
    private double x;
    private double y;
    private double heading;

    public Pose(double setX, double setY, double setHeading) {
        setX(setX);
        setY(setY);
        setHeading(setHeading);
    }

    public Pose(double setX, double setY) {
        this(setX, setY, 0);
    }

    public Pose() {
        this(0,0,0);
    }

    public void setX(double set) {
        x = set;
    }

    public void setY(double set) {
        y = set;
    }

    public void setHeading(double set) {
        heading = MathFunctions.normalizeAngle(set);
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeading() {
        return heading;
    }

    public Vector getVector() {
        Vector returnVector = new Vector();
        returnVector.setOrthogonalComponents(x, y);
        return returnVector;
    }

    public Vector getHeadingVector() {
        return new Vector(1, heading);
    }

    public void add(Pose pose) {
        setX(x + pose.getX());
        setY(y + pose.getY());
        setHeading(heading + pose.getHeading());
    }

    public void subtract(Pose pose) {
        setX(x - pose.getX());
        setY(y - pose.getY());
        setHeading(heading - pose.getHeading());
    }

    public void scalarMultiply(double scalar) {
        setX(x * scalar);
        setY(y * scalar);
        setHeading(heading * scalar);
    }

    public void scalarDivide(double scalar) {
        setX(x / scalar);
        setY(y / scalar);
        setHeading(heading / scalar);
    }

    public void flipSigns() {
        setX(-x);
        setY(-y);
        setHeading(-heading);
    }

    public boolean roughlyEquals(Pose pose, double accuracy) {
        return MathFunctions.roughlyEquals(x, pose.getX(), accuracy) && MathFunctions.roughlyEquals(y, pose.getY(), accuracy) && MathFunctions.roughlyEquals(MathFunctions.getSmallestAngleDifference(heading, pose.getHeading()), 0, accuracy);
    }

    public boolean roughlyEquals(Pose pose) {
        return roughlyEquals(pose, 0.0001);
    }

    public Pose copy() {
        return new Pose(getX(), getY(), getHeading());
    }
}
