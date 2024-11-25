package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.ImuCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.ImuSubsystem;

public class ImuSetYawCommand extends CommandBase {
    private ImuSubsystem imuSubsystem;
    private ImuResetCommand imuResetCommand;
    private double currentYaw;
    private Telemetry telemetry;
    public ImuSetYawCommand(ImuSubsystem imuSubsystem, ImuResetCommand imuResetCommand, double currentYaw, Telemetry telemetry){
        this.imuSubsystem = imuSubsystem;
        this.imuResetCommand = imuResetCommand;
        this.currentYaw = currentYaw;
        this.telemetry = telemetry;
        addRequirements(imuSubsystem);
    }
    @Override
    public void execute(){
        telemetry.addData("current Yaw", currentYaw);
        telemetry.update();
    }
    @Override
    public boolean isFinished() {
        return true;
    }
}
