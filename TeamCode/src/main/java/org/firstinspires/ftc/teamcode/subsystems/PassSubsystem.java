package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PassSubsystem extends SubsystemBase {
    private DcMotorEx motor1;

    public PassSubsystem(DcMotorEx motor1) {
        this.motor1 = motor1;
    }
    public void PassMotorControl(double motorPower){
        motor1.setPower(motorPower);
    }
    public void PassOn(){
        motor1.setPower(0.75);
    }
    public void PassOff(){
        motor1.setPower(0);
    }
}
