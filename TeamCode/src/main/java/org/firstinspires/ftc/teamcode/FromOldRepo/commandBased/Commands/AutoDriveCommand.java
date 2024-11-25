package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

public class AutoDriveCommand extends CommandBase {
    private AutoDriveSubsystem autoDriveSubsystem;
    private Telemetry telemetry;
    public AutoDriveCommand(AutoDriveSubsystem autoDriveSubsystem, Telemetry telemetry){
        this.autoDriveSubsystem = autoDriveSubsystem;
        this.telemetry = telemetry;
        addRequirements(autoDriveSubsystem);
    }

    @Override
    public void execute(){
        autoDriveSubsystem.update();
        autoDriveSubsystem.telemetryDebug(telemetry);
    }
    @Override
    public boolean isFinished(){
        return !autoDriveSubsystem.isBusy();
    }
    public void setPath(Path path, boolean holdEnd){
        autoDriveSubsystem.followPath(path, holdEnd);
    }
    public void setPathChain(PathChain pathChain, boolean holdEnd){
        autoDriveSubsystem.followPath(pathChain, holdEnd);
    }
}
