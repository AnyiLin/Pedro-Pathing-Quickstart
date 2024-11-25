package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.ExtensionSubsystem;

public class ExtensionOutCommand extends CommandBase {
    private ExtensionSubsystem extensionSubsystem;

    public ExtensionOutCommand(ExtensionSubsystem extensionSubsystem) {
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