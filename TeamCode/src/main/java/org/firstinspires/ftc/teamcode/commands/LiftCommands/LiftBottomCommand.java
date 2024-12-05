package org.firstinspires.ftc.teamcode.commands.LiftCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftBottomCommand extends CommandBase {
    private LiftSubsystem liftSubsystem;
    private Telemetry telemetry;
    public LiftBottomCommand (LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);
    }
    @Override
    public void execute () {
        liftSubsystem.setBottomPosition();
        liftSubsystem.runLift();
    }
    @Override
    public boolean isFinished () { return liftSubsystem.isBusy(); }
}
