package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.PedroDriveSubsystem;

public class CameraAdjustTeleCommand extends CommandBase {
    private PedroDriveSubsystem pedroDriveSubsystem;

    public CameraAdjustTeleCommand(PedroDriveSubsystem pedroDriveSubsystem) {
        this.pedroDriveSubsystem = pedroDriveSubsystem;
        addRequirements(pedroDriveSubsystem);
    }

    @Override
    public void execute() {
//        pedroDriveSubsystem.cameraAdjust();
    }
    @Override
    public boolean isFinished(){
        return true;
    }

}
