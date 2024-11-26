package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;

public class PracticeOpMode extends CommandOpMode {
    private ExtendSubsystem extend;
    private GamepadEx driverOp;
    @Override
    public void initialize() {
        extend = new ExtendSubsystem(hardwareMap.get(Servo.class, "extension"));
        driverOp = new GamepadEx(gamepad1);

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ExtendCommand(extend));
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new RetractCommand(extend));
    }
}
