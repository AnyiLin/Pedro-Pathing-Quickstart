package org.firstinspires.ftc.teamcode.commands.SwingArmCommand;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;

public class SwingArmMidCommand  extends CommandBase {
    private SwingArmSubsystem SwingArmSubsystem;

    public SwingArmMidCommand(SwingArmSubsystem SwingArmSubsystem) {
        this.SwingArmSubsystem = SwingArmSubsystem;
        addRequirements(SwingArmSubsystem);
    }

    @Override
    public void initialize() {
        SwingArmSubsystem.mid();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
