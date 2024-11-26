package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.oldexten;

public class ExtensionOutCommand extends CommandBase {
    private oldexten extensionSubsystem;

    public ExtensionOutCommand(oldexten extensionSubsystem) {
        this.extensionSubsystem = extensionSubsystem;
        addRequirements(extensionSubsystem);
    }

    @Override
    public void execute() {
        extensionSubsystem.setExtensionOut();
        extensionSubsystem.runExtension();
    }

    @Override
    public boolean isFinished() {
        return extensionSubsystem.isExtensionBusy();
    }
}