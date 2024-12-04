package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class WristSubsystem extends SubsystemBase {
    private Servo servo;
    public WristSubsystem(Servo servo) {
        this.servo = servo;
    }
    public void up() {
        servo.setPosition(1.0);
    }
    public void down() {
        servo.setPosition(0.0);
    }
}
