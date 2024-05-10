package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

/**
 * This is the Localizer class. It is an abstract superclass of all localizers used in Pedro Pathing,
 * so it contains abstract methods that will have a concrete implementation in the subclasses. Any
 * method that all localizers will need will be in this class.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */
public abstract class Localizer {

    public abstract Pose getPose();

    public abstract Pose getVelocity();

    public abstract Vector getVelocityVector();

    public abstract void setStartPose(Pose setStart);

    public abstract void setPose(Pose setPose);

    public abstract void update();

    public abstract double getTotalHeading();

    public abstract double getForwardMultiplier();

    public abstract double getLateralMultiplier();

    public abstract double getTurningMultiplier();
}
