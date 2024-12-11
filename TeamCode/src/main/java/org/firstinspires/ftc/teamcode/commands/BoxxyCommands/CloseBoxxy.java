package org.firstinspires.ftc.teamcode.commands.BoxxyCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;

public class CloseBoxxy extends CommandBase {
    private BoxxySubsystem subsystem;

    public CloseBoxxy(BoxxySubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.close();
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}
