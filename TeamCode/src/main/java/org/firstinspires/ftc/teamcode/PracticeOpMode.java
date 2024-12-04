package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.commands.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.CancelCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.CollectSample;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.EjectCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.ToggleAlliance;
import org.firstinspires.ftc.teamcode.commands.LowerWrist;
import org.firstinspires.ftc.teamcode.commands.RaiseWrist;
import org.firstinspires.ftc.teamcode.commands.RetractCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@TeleOp
public class PracticeOpMode extends CommandOpMode {
    private ExtendSubsystem extend;
    private IntakeSubsystem intake;
    private GamepadEx driverOp;
    private GamepadEx Optwo;
    private WristSubsystem wrist;

    @Override
    public void initialize() {
        extend = new ExtendSubsystem(hardwareMap.get(Servo.class, "extension"));
        wrist = new WristSubsystem(hardwareMap.get(Servo.class, "wrist"));
        driverOp = new GamepadEx(gamepad1);
        Optwo = new GamepadEx(gamepad2);
        intake = new IntakeSubsystem(telemetry, hardwareMap.get(DcMotor.class, "Intake"),
                hardwareMap.get(ColorSensor.class, "intakeColor"),
                hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"),
                hardwareMap.get(DistanceSensor.class, "intakeDistance"),
                hardwareMap.get(ServoImplEx.class, "allianceColor"));


        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ExtendCommand(extend));
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new RetractCommand(extend));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new RaiseWrist(wrist));
        driverOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new LowerWrist(wrist));
        driverOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new EjectCommand(intake));
        driverOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new CancelCommand(intake));
        driverOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ToggleAlliance(intake));
        driverOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new CollectSample(intake));
        intake.rainbowlight();
    }
}
