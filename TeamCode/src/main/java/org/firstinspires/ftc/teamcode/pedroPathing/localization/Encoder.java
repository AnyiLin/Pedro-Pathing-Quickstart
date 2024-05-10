package org.firstinspires.ftc.teamcode.pedroPathing.localization;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This is the Encoder class.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/2/2024
 */
public class Encoder {
    private DcMotorEx motor;
    private double previousPosition;
    private double currentPosition;
    private double multiplier;

    public final static double FORWARD = 1, REVERSE = -1;

    public Encoder(DcMotorEx setMotor) {
        motor = setMotor;
        multiplier = FORWARD;
        reset();
    }

    public void setDirection(double setMultiplier) {
        multiplier = setMultiplier;
    }

    public void reset() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        previousPosition = motor.getCurrentPosition();
        currentPosition = motor.getCurrentPosition();
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        previousPosition = currentPosition;
        currentPosition = motor.getCurrentPosition();
    }

    public double getMultiplier() {
        return multiplier * (motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1);
    }

    public double getDeltaPosition() {
        return getMultiplier() * (currentPosition - previousPosition);
    }
}
