package org.firstinspires.ftc.teamcode.commands.WristCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class HandoffCommand extends CommandBase {
    private WristSubsystem subsystem;
    public HandoffCommand(WristSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() { subsystem.middle();}
    @Override
    public boolean isFinished() { return true;}
}
