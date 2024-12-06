package org.firstinspires.ftc.teamcode.commands.SwingArmCommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class SwingArmDownCommand extends CommandBase {
    private SwingArmSubsystem SwingArmSubsystem;
    public SwingArmDownCommand(SwingArmSubsystem SwingArmSubsystem) {
        this.SwingArmSubsystem = SwingArmSubsystem;
        addRequirements(SwingArmSubsystem);
    }
    @Override
    public void initialize() { SwingArmSubsystem.down(); }
    @Override
    public boolean isFinished() { return true; }
}