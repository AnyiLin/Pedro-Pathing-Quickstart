package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class LocalizerSubsystem extends SubsystemBase {
    private Telemetry telemetry;
    private LimelightSubsystem limelightSubsystem;
    private OdometrySubsystem odometrySubsystem;
    private double yaw = 0;
    private boolean yawCorrectionSet = false;
    public boolean posCorrectionSet = false;
    Pose2D localizerPose;
    double xOffset = 0;
    double yOffset = 0;
    double headingOffset = 0;


    public LocalizerSubsystem(Telemetry telemetry, LimelightSubsystem limelightSubsystem, OdometrySubsystem odometrySubsystem){
        this.telemetry = telemetry;
        this.limelightSubsystem = limelightSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        localizerPose = odometrySubsystem.getOdometryPose();
    }
    public void getLocalizerTelemetry(){
        telemetry.addData("Pos x", getLocalizerPose().getX(DistanceUnit.INCH));
        telemetry.addData("Pos y", getLocalizerPose().getY(DistanceUnit.INCH));
        telemetry.addData("Pos heading", getLocalizerPose().getHeading(AngleUnit.DEGREES));
    }
    public double getLimelightHeading(){
        return limelightSubsystem.getYawAprilTag();
    }
    public double getOdoHeading(){
        return odometrySubsystem.getHeadingOdo();
    }
    public Pose2D getLocalizerPose(){
//        localizerPose = new Pose2D(DistanceUnit.INCH, odometrySubsystem.getOdometryPose().getX(DistanceUnit.INCH), odometrySubsystem.getOdometryPose().getY(DistanceUnit.INCH), AngleUnit.RADIANS, odometrySubsystem.getOdometryPose().getHeading(AngleUnit.RADIANS));
        localizerPose = odometrySubsystem.getOdometryPose();
        return localizerPose;
    }
    public Pose2D getLocalizerVelocity(){
        return odometrySubsystem.getOdometryVelocity();
    }
    public void update(){
        odometrySubsystem.updateOdometry();
    }
    public void setLocalizerPose(Pose2D pose){
        odometrySubsystem.setOdoPos(pose);
    }
    public Pose cameraAjust(){
        LLResult limeLightResult = limelightSubsystem.readAprilTag();
        Pose2D limeLightPose = new Pose2D(DistanceUnit.METER, limeLightResult.getBotpose().getPosition().y, limeLightResult.getBotpose().getPosition().x, AngleUnit.RADIANS, limeLightResult.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS));
        telemetry.addData("LL Result x", limeLightPose.getX(DistanceUnit.INCH));
        telemetry.addData("LL Result y", limeLightPose.getY(DistanceUnit.INCH) );
        telemetry.addData("LL Result yaw", limeLightPose.getHeading(AngleUnit.DEGREES));
        telemetry.update();
        // Lime light x and y are swapped compared to odometry x and y

        return new Pose(limeLightPose.getX(DistanceUnit.INCH), limeLightPose.getY(DistanceUnit.INCH), limeLightPose.getHeading(AngleUnit.RADIANS));
    }


}
