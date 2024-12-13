package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.IntakeCommands.EjectCommand;
import org.firstinspires.ftc.teamcode.commands.PassCommands.PassOnCommand;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

public class PassAuto extends ParallelCommandGroup {
    public PassAuto(PassSubsystem pass, BoxxySubsystem box, IntakeSubsystem intake){
        addCommands(
                new EjectCommand(intake),
                new PassOnCommand(pass,box)
        );
    }
}
