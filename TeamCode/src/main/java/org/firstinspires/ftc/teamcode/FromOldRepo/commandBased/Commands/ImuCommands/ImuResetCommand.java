package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.ImuCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.ImuSubsystem;

public class ImuResetCommand extends CommandBase {
    private final ImuSubsystem imuSubsystem;
       public ImuResetCommand(ImuSubsystem imuSubsystem){
        this.imuSubsystem = imuSubsystem;
        addRequirements(imuSubsystem);
    }
    @Override
    public void initialize(){
        imuSubsystem.resetYaw();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
