package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class BoxxySubsystem extends SubsystemBase {
    private Servo servo;
    public BoxxySubsystem(Servo servo) {
        this.servo = servo;
    }
    public void open() {
        servo.setPosition(1.0);

    }
    public void close() {
        servo.setPosition(0.5);
    }
}
