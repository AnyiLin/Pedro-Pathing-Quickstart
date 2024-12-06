package org.firstinspires.ftc.teamcode.commands.LiftCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftTelemetryCommand extends CommandBase {
    private LiftSubsystem liftSubsystem;
    public LiftTelemetryCommand (LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);
    }
    @Override
    public void execute () {
        liftSubsystem.getLiftTelemetry();

    }
}
