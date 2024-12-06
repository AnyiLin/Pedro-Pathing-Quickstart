package org.firstinspires.ftc.teamcode.commands.SwingArmCommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class SwingArmUpCommand extends CommandBase {
    private SwingArmSubsystem SwingArmSubsystem;
    public SwingArmUpCommand(SwingArmSubsystem SwingArmSubsystem) {
        this.SwingArmSubsystem = SwingArmSubsystem;
        addRequirements(SwingArmSubsystem);
    }
    @Override
    public void initialize() { SwingArmSubsystem.up(); }
    @Override
    public boolean isFinished() { return true; }
}