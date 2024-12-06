package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;

public class OpenBox extends CommandBase {
    private BoxxySubsystem subsystem;
    public OpenBox(BoxxySubsystem subsystem) {
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {subsystem.open();}
    @Override
    public boolean isFinished() {return true;}
}
