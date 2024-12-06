package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.CloseBox;
import org.firstinspires.ftc.teamcode.commands.OpenBox;
import org.firstinspires.ftc.teamcode.subsystems.BoxxySubsystem;

@TeleOp

public class BoxxyTest extends CommandOpMode {
    private BoxxySubsystem subsystem;
    private GamepadEx gamepadEx;
    @Override
    public void initialize() {
        subsystem = new BoxxySubsystem( hardwareMap.get(Servo.class, "Boxxy"));
        gamepadEx = new GamepadEx(gamepad1);
        gamepadEx.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new OpenBox(subsystem));
        gamepadEx.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new CloseBox(subsystem));

    }
}

