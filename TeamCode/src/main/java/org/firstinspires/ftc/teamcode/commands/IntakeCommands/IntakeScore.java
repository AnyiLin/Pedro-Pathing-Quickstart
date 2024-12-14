package org.firstinspires.ftc.teamcode.commands.IntakeCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.PassAuto;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmMidCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.HandoffCommand;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class IntakeScore extends SequentialCommandGroup {
    private IntakeSubsystem intake;
    private WristSubsystem wrist;
    private PassSubsystem pass;
    private LiftSubsystem lift;
    private ExtendSubsystem extend;
    private SwingArmSubsystem swingArm;
    private BoxxySubsystem box;

    public IntakeScore(IntakeSubsystem intake, WristSubsystem wrist, PassSubsystem pass, ExtendSubsystem extend, SwingArmSubsystem swingArm, BoxxySubsystem box, LiftSubsystem lift){
        addCommands(
                new CollectSample(intake),
                new HandoffCommand(wrist),
                new RetractCommand(extend),
                new PassAuto(pass, box, intake),
                new LiftTopCommand(lift),
                new SwingArmMidCommand(swingArm)
        );
    }
}
