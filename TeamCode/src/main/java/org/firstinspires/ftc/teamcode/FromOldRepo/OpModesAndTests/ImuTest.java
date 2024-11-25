package org.firstinspires.ftc.teamcode.FromOldRepo.OpModesAndTests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.ImuCommands.GetYawCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.ImuCommands.ImuResetCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.ImuCommands.ImuSetYawCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.ImuSubsystem;

@Disabled
@TeleOp(name = "ImuTest1")
public class ImuTest extends CommandOpMode {
    private IMU imu;
    private RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);;
    private ImuSubsystem imuSubsystem;
    private GetYawCommand getYawCommand;
    private ImuResetCommand imuResetCommand;
    private ImuSetYawCommand imuSetYawCommand;
    private GamepadEx driverOp;
    private Button aButton, bButton;
    @Override
    public void initialize(){
        driverOp = new GamepadEx(gamepad1);
        imu = hardwareMap.get(IMU.class, "imu");
        imuSubsystem = new ImuSubsystem(imu, orientationOnRobot, telemetry);
        getYawCommand = new GetYawCommand(imuSubsystem);
        imuResetCommand = new ImuResetCommand(imuSubsystem);
        imuSetYawCommand = new ImuSetYawCommand(imuSubsystem, imuResetCommand, 43, telemetry);
        aButton = (new GamepadButton(driverOp, GamepadKeys.Button.A))
                .whenPressed(imuResetCommand);//should reset Imu's yaw when a is pressed
        bButton = (new GamepadButton(driverOp, GamepadKeys.Button.B))
                .whenPressed(imuSetYawCommand);
        register(imuSubsystem);
        imuSubsystem.setDefaultCommand(getYawCommand);//Should return yaw
    }
}
