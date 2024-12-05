package org.firstinspires.ftc.teamcode.subsystems;

//import static org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem.liftPosition.TOP;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftSubsystem extends SubsystemBase {
    private Motor liftMotor = null;
    private int topPosition = 1000;
    private Telemetry telemetry;
    private double power = 1;

    public enum liftPosition {
        TOP,
        BOTTOM

    }
    public liftPosition liftPos;

    public LiftSubsystem (Motor liftMotor, Telemetry telemetry) {
        this.liftMotor = liftMotor;
        this.telemetry = telemetry;
        this.liftMotor.setRunMode(Motor.RunMode.PositionControl);
        this.liftMotor.resetEncoder();
        this.liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        liftPos = liftPosition.BOTTOM;

    }
    public void setTopPosition () {
        liftMotor.setTargetPosition(topPosition);
        liftPos = liftPosition.TOP;
    }
    public void setBottomPosition () {
        liftMotor.setTargetPosition(0);
        liftPos = liftPosition.BOTTOM;
    }
    public boolean isBusy () {
        return !liftMotor.atTargetPosition();
    }
    public void runLift () {
        if (isBusy()) {
            liftMotor.set(power);
        } else {
            liftMotor.stopMotor();
        }
    }
}
