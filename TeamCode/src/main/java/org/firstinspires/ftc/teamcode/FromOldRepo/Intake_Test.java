package org.firstinspires.ftc.teamcode.FromOldRepo;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.IntakeCommands.CancelCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.IntakeCommands.CollectSample;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.IntakeCommands.EjectCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.IntakeCommands.ToggleAlliance;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.IntakeSubsystem;
@TeleOp
@Disabled
public class Intake_Test extends CommandOpMode {
    private IntakeSubsystem subsystem;
    private GamepadEx gamepad;
    @Override
    public void initialize() {
        subsystem = new IntakeSubsystem(telemetry, hardwareMap.get(DcMotor.class, "Intake"),
                hardwareMap.get(ColorSensor.class, "intakeColor"),
                hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"),
                hardwareMap.get(DistanceSensor.class, "intakeDistance"),
                hardwareMap.get(ServoImplEx.class, "allianceColor"));
        gamepad = new GamepadEx(gamepad1);

        gamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new EjectCommand(subsystem));
        gamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new CancelCommand(subsystem));
        gamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ToggleAlliance(subsystem));
        gamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new CollectSample(subsystem));
        subsystem.rainbowlight();


    }
}
