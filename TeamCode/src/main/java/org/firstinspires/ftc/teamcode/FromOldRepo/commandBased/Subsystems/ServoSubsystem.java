package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoSubsystem extends SubsystemBase {
    private CRServo testservo;
    private final double servospeed = 1;
    private RevTouchSensor toucher;
    private RevColorSensorV3 coloreader;
    public boolean isTouchSensorPressed() {
        return toucher.isPressed();
    }

    public ServoSubsystem(final HardwareMap hardwareMap, final String name, final String name1, final String name2) {
        testservo = hardwareMap.get(CRServo.class, name);
        toucher = hardwareMap.get(RevTouchSensor.class, name1);
        coloreader = hardwareMap.get(RevColorSensorV3.class, name2);
    }

//    public void runServo() {
//        if (toucher.isPressed()) {
//            servostop();
//        } else {
//            servorunf();
//        }
//    }
    public boolean touchsensor() {
        if (toucher.isPressed()) {
            return true;
        } else {
            return false;
        }
    }

    public void servorunf() {
        testservo.setPower(servospeed);
    }

    public void servorunb() {
        testservo.setPower(-servospeed);
    }

    public void servostop() {
        testservo.setPower(0);
    }
}
