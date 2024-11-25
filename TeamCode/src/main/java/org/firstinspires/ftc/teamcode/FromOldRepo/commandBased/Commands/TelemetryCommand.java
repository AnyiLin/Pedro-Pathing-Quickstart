package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.TelemetrySubsystem;

public class TelemetryCommand extends CommandBase {
    private TelemetrySubsystem telemetrySubsystem;

    public TelemetryCommand(TelemetrySubsystem telemetrySubsystem){
        this.telemetrySubsystem = telemetrySubsystem;
        addRequirements(telemetrySubsystem);
    }
    @Override
    public void execute(){telemetrySubsystem.getTelemetry();}

}
