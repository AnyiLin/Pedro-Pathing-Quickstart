package org.firstinspires.ftc.teamcode.FromOldRepo;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.Servo_CommandRun;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.Servo_CommandStop;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.ServoSubsystem;


@TeleOp(name="Servo Test OpMode")
@Disabled
public class Servo_Test extends CommandOpMode {

    private ServoSubsystem m_servoSubsystem;
    private Servo_CommandRun m_runServoCommand;
    private Servo_CommandStop m_stopServoCommand;
    private RevTouchSensor toucher;
    private GamepadEx gamepadA;

@Override
    public void initialize() {
    gamepadA= new GamepadEx(gamepad1);
        m_servoSubsystem = new ServoSubsystem(hardwareMap, "Test", "Touch", "color"); // Replace with actual device names
        m_runServoCommand = new Servo_CommandRun(m_servoSubsystem);

        m_runServoCommand.execute();
        schedule(m_runServoCommand);

    gamepadA.getGamepadButton(GamepadKeys.Button.A)
            .whenPressed(m_runServoCommand)
            .whenReleased(m_stopServoCommand);
    }
}










// Servo= new ServoSubsystem(hardwareMap, "Test", "Touch","color" );