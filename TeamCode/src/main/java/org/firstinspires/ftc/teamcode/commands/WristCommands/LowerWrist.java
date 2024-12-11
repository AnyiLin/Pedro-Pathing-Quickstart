package org.firstinspires.ftc.teamcode.commands.WristCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class LowerWrist extends CommandBase {
    private WristSubsystem subsystem;
    public LowerWrist(WristSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        subsystem.down();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
