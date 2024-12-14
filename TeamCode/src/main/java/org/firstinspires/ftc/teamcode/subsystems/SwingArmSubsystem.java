package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class SwingArmSubsystem extends SubsystemBase {
    private Servo SwingArmServo;
    private double TopPos = 0.75;
    private double MidPos = 0.35;
    public SwingArmSubsystem(Servo SwingArmServo) {
        this.SwingArmServo = SwingArmServo;
        this.SwingArmServo.setDirection(Servo.Direction.REVERSE);
    }
    public void up() {
        SwingArmServo.setPosition(TopPos);
    }
    public void down() {
        SwingArmServo.setPosition(0.0);
    }
    public void mid() { SwingArmServo.setPosition(MidPos);}

}