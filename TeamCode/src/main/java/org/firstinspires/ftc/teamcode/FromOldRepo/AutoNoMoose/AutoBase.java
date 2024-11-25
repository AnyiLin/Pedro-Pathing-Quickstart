package org.firstinspires.ftc.teamcode.FromOldRepo.AutoNoMoose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

public abstract class AutoBase extends LinearOpMode {
    public Telemetry mTelemetry;
    protected Follower follower;
    protected Path path1;
    protected Path path2;
    protected Path path3;

    protected Pose pose1 = new Pose(20, -20, 0);
    protected Pose startPose = new Pose(0, 0, 0);

    public void initializeAuto(Telemetry telemetry, HardwareMap hardwareMap) {

        this.mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }
    public void runOpMode() throws InterruptedException {

    }
}
