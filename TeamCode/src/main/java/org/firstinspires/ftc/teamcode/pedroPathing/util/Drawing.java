package org.firstinspires.ftc.teamcode.pedroPathing.util;

import com.acmerobotics.dashboard.canvas.Canvas;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

/*
* Drawing class needed for putting your robot onto the dashboard
* Edit the radius needed to align to your robot the best possible simulation represented
 */
public class Drawing {
    private Drawing() {}

    public static void drawRobot(Canvas c, Point t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);

        Vector halfv = new Vector(0.5*ROBOT_RADIUS, t.getTheta());
        Vector p1 = MathFunctions.addVectors(halfv, new Vector(t.getR(), t.getTheta()));
        Vector p2 = MathFunctions.addVectors(p1, halfv);
        c.strokeLine(p1.getXComponent(), p1.getYComponent(), p2.getXComponent(), p2.getYComponent());
    }

    public static void drawRobot(Canvas c, Pose t) {
        final double ROBOT_RADIUS = 9;

        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);
        Vector v = t.getHeadingVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = t.getX() + v.getXComponent() / 2, y1 = t.getY() + v.getYComponent() / 2;
        double x2 = t.getX() + v.getXComponent(), y2 = t.getY() + v.getYComponent();
        c.strokeLine(x1, y1, x2, y2);
    }
}
