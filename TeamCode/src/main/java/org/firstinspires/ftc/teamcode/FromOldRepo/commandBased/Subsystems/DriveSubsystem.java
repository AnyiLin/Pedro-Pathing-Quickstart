package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

public class DriveSubsystem extends SubsystemBase {
    private MecanumDrive drive;
    private Motor frontLeft, frontRight, backLeft, backRight;
    private Telemetry telemetry;
    public DriveSubsystem(Motor frontLeft, Motor frontR, Motor backL, Motor backR, Telemetry telemetry){
        this.frontLeft = frontLeft;
        this.frontRight = frontR;
        this.backLeft = backL;
        this.backRight = backR;
        this.drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        this.telemetry = telemetry;
    }
    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, double gyroAngle, boolean fieldCentric){
        if(fieldCentric) {
            drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, gyroAngle);
        }else {
            drive.driveRobotCentric(strafeSpeed, forwardSpeed, turnSpeed);
        }

    }
    public void getDriveTelemetry(){
    }
}
