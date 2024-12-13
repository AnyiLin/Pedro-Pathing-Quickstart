package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

public class CancelCommand extends CommandBase {
    private IntakeSubsystem subsystem;
    private PassSubsystem pass;
    public CancelCommand(IntakeSubsystem subsystem, PassSubsystem pass) {
        this.subsystem = subsystem;
        this.pass = pass;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        subsystem.stopMotor();
        pass.PassOff();

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}