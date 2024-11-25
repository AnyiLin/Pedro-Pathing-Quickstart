package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.ImuCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.ImuSubsystem;


public class GetYawCommand extends CommandBase {
    private final ImuSubsystem imuSubsystem;
    public GetYawCommand(ImuSubsystem imuSubsystem){
        this.imuSubsystem = imuSubsystem;
        addRequirements(imuSubsystem);
    }
    public void initialize(){
        imuSubsystem.getImuValues();
    }

    @Override
    public void execute(){
        imuSubsystem.getImuYawDeg();
    }

}
