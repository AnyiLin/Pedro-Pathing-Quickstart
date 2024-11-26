package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;

public class ExtendCommand extends CommandBase {
    private ExtendSubsystem subsystem;

    public ExtendCommand(ExtendSubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

        @Override
        public void initialize () {
            subsystem.extend();
        }

        @Override
    public boolean isFinished() { return true; }
}
