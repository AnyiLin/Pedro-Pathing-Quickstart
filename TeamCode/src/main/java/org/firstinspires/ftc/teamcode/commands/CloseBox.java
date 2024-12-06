package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;

public class CloseBox extends CommandBase {
    private BoxxySubsystem subsystem;

    public CloseBox(BoxxySubsystem subsystem) {
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
