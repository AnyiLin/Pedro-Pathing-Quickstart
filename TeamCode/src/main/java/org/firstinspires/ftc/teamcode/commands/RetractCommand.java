package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;

public class RetractCommand extends CommandBase {
    private ExtendSubsystem subsystem;
     public RetractCommand(ExtendSubsystem subsystem) {
         this.subsystem = subsystem;
         addRequirements(subsystem);
     }
     @Override
    public void initialize() {
    subsystem.retract();
     }
     @Override
    public boolean isFinished() { return true; }
}
