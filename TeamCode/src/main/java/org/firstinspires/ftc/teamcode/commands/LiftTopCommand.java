package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class LiftTopCommand extends CommandBase {
    private LiftSubsystem liftSubsystem;
    private Telemetry telemetry;
    public LiftTopCommand (LiftSubsystem liftSubsystem) {
        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);
    }
    @Override
    public void execute () {
        liftSubsystem.setTopPosition();
        liftSubsystem.runLift();
    }
    @Override
    public boolean isFinished () {
        return liftSubsystem.isBusy();
    }
}
