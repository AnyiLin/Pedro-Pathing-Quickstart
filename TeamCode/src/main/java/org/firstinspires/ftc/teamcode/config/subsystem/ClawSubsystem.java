package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class ClawSubsystem {

    private Servo pivot, clawL, clawR;

    public ClawSubsystem(HardwareMap hardwareMap) {
        pivot = hardwareMap.get(Servo.class, "pivot");
        clawL = hardwareMap.get(Servo.class, "clawL");
        clawR = hardwareMap.get(Servo.class, "clawR");
    }

    //------------------------------Close Claws------------------------------//

    public void closeLClaw() {
        clawL.setPosition(RobotConstants.closedL);
    }

    public void closeRClaw() {
        clawR.setPosition(RobotConstants.closedR);
    }

    public void closeClaws() {
        clawL.setPosition(RobotConstants.closedL);
        clawR.setPosition(RobotConstants.closedR);
    }

    //------------------------------Open Claws------------------------------//
    public void openLClaw() {
        clawL.setPosition(RobotConstants.openL);
    }

    public void openRClaw() {
        clawR.setPosition(RobotConstants.openR);
    }

    public void openClaws() {
        clawL.setPosition(RobotConstants.openL);
        clawR.setPosition(RobotConstants.openR);
    }

    //------------------------------Claw Rotate------------------------------//

    public void startClaw() {
        pivot.setPosition(RobotConstants.startClaw);
    }

    public void groundClaw() {
        pivot.setPosition(RobotConstants.groundClaw);
    }

    public void scoringClaw() {
        pivot.setPosition(RobotConstants.scoringClaw);
    }


}