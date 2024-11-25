package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.ServoSubsystem;

public class Servo_CommandStop extends CommandBase {

    private ServoSubsystem m_servoSubsystem;

    public Servo_CommandStop(ServoSubsystem servoSubsystem) {
        m_servoSubsystem = servoSubsystem;
        addRequirements(servoSubsystem);
    }

    @Override
    public void execute() {
        m_servoSubsystem.servostop();
    }

//    @Override
//    public void end(boolean interrupted) {
//        m_servoSubsystem.servostop();
//    }

    @Override
    public boolean isFinished() {
        return false; // Change this condition if you want the command to end automatically
    }


}
