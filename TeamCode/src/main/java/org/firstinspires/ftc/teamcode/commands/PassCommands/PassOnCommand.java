package org.firstinspires.ftc.teamcode.commands.PassCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;

public class PassOnCommand extends SequentialCommandGroup {
    private PassSubsystem pass;
    private BoxxySubsystem box;
    public PassOnCommand(PassSubsystem pass, BoxxySubsystem box){
        this.pass = pass;
        this.box = box;
        addRequirements(pass, this.box);
    }

    @Override
    public void initialize(){pass.PassOn();}

    public void end(){pass.PassOff();}
    @Override
    public boolean isFinished(){return box.haveSample();}


}
