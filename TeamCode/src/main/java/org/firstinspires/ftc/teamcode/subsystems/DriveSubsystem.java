package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DriveSubsystem extends SubsystemBase {
    private MecanumDrive drive;
    private Motor frontleft, frontRight, backLeft, backRight;
    public DriveSubsystem(Motor frontleft, Motor frontRight, Motor backLeft, Motor backRight) {
        this.frontleft = frontleft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.drive = new MecanumDrive(false, frontleft, frontRight,backLeft,backRight);
    }
    public void drive(double strafe, double forward, double turn) {
        frontleft.setInverted(true);
        frontRight.setInverted(false);
        backLeft.setInverted(true);
        backRight.setInverted(false);
        drive.driveRobotCentric(strafe,forward,turn);
    }
}
