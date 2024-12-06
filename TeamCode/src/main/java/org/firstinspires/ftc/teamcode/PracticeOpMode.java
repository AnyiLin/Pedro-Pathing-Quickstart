package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTelemetryCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.RetractCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

public class PracticeOpMode extends CommandOpMode {
    private ExtendSubsystem extend;
    private LiftSubsystem liftSubsystem;

    private GamepadEx driverOp;
    private Motor leftFront, rightFront, leftRear, rightRear, liftMotor;

    @Override
    public void initialize() {
        liftMotor = new Motor(hardwareMap, "liftMotor");
        extend = new ExtendSubsystem(hardwareMap.get(Servo.class, "extension"));
        liftSubsystem = new LiftSubsystem(liftMotor, telemetry);

        driverOp = new GamepadEx(gamepad1);
        liftSubsystem.setDefaultCommand(new LiftTelemetryCommand(liftSubsystem));
        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new LiftTopCommand(liftSubsystem));
        driverOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new LiftBottomCommand(liftSubsystem));
        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ExtendCommand(extend));
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new RetractCommand(extend));
    }
}
