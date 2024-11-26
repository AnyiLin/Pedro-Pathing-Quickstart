package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class CancelCommand extends CommandBase {
    private IntakeSubsystem subsystem;
    public CancelCommand(IntakeSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        subsystem.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}