package org.firstinspires.ftc.teamcode.commands.PassCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

import java.util.function.DoubleSupplier;

public class PassCommand extends CommandBase {
    private PassSubsystem pass;
    private DoubleSupplier motorPower;
    public PassCommand(PassSubsystem pass, DoubleSupplier motorPower){
        this.pass = pass;
        this.motorPower =motorPower;
        addRequirements(pass);
    }
    @Override
    public void execute() {
        pass.PassMotorControl(motorPower.getAsDouble());
    }
    @Override
    public boolean isFinished(){return false;}
}
