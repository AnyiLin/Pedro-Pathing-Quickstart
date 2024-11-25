package org.firstinspires.ftc.teamcode.FromOldRepo.OpModesAndTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.AutoDriveCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.VisionCommands.CameraAjustCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.LocalizerSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public abstract class FtcLibAutoBase extends CommandOpMode {
    protected Follower follower;
    protected PathChain pathChain;
    protected Telemetry mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    protected Limelight3A  limelight3A;
    protected AutoDriveSubsystem autoDriveSubsystem;
    protected AutoDriveCommand autoDriveCommand;
    protected CameraAjustCommand cameraAjustCommand;
    protected LimelightSubsystem limelightSubsystem;


}
