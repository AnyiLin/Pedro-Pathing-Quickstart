package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.CancelCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.CollectSample;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.EjectCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommands.ToggleAlliance;
import org.firstinspires.ftc.teamcode.commands.PassCommands.PassCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftBottomCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTelemetryCommand;
import org.firstinspires.ftc.teamcode.commands.LiftCommands.LiftTopCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommands.RetractCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmDownCommand;
import org.firstinspires.ftc.teamcode.commands.SwingArmCommand.SwingArmUpCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.HandoffCommand;
import org.firstinspires.ftc.teamcode.commands.WristCommands.LowerWrist;
import org.firstinspires.ftc.teamcode.commands.WristCommands.RaiseWrist;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtendSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PassSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SwingArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@TeleOp(name = "PracticeOpMode")
public class PracticeOpMode extends CommandOpMode {
    private ExtendSubsystem extend;
    private LiftSubsystem liftSubsystem;
    private SwingArmSubsystem swingArmSubsystem;
    private PassSubsystem pass;
    private IntakeSubsystem intake;
//-8300
    private GamepadEx driverOp;
    private GamepadEx operatorOp;
    private Motor liftMotor;
    private DriveSubsystem drive;
    private WristSubsystem wrist;

    @Override
    public void initialize() {
        driverOp = new GamepadEx(gamepad1);
        operatorOp = new GamepadEx(gamepad2);
        liftMotor = new Motor(hardwareMap, "liftMotor", Motor.GoBILDA.RPM_117);
        extend = new ExtendSubsystem(hardwareMap.get(Servo.class, "extension"));
        swingArmSubsystem = new SwingArmSubsystem(hardwareMap.get(Servo.class, "swingArm"));
        liftSubsystem = new LiftSubsystem(liftMotor, telemetry);
        pass = new PassSubsystem(hardwareMap.get(DcMotorEx.class, "pass"));
        wrist = new WristSubsystem(hardwareMap.get(Servo.class,"wrist"));
        drive = new DriveSubsystem(
                new Motor(hardwareMap,"frontLeft", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap,"frontRight", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap,"backLeft", Motor.GoBILDA.RPM_312),
                new Motor(hardwareMap,"backRight", Motor.GoBILDA.RPM_312));
        liftSubsystem.setDefaultCommand(new LiftTelemetryCommand(liftSubsystem));

        intake = new IntakeSubsystem(telemetry, hardwareMap.get(DcMotor.class, "Intake"),
                hardwareMap.get(ColorSensor.class, "intakeColor"),
                hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"),
                hardwareMap.get(DistanceSensor.class, "intakeDistance"),
                hardwareMap.get(ServoImplEx.class, "allianceColor"));

        drive.setDefaultCommand(new DriveCommand(drive, driverOp::getLeftX,driverOp::getLeftY,driverOp::getRightX));
        

        operatorOp.getGamepadButton(GamepadKeys.Button.A)
                .toggleWhenPressed(new LiftTopCommand(liftSubsystem), new LiftBottomCommand(liftSubsystem));
        operatorOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new RaiseWrist(wrist));
        operatorOp.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(new SwingArmUpCommand(swingArmSubsystem), new SwingArmDownCommand(swingArmSubsystem));
        operatorOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new LowerWrist(wrist));
        operatorOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new HandoffCommand(wrist))
                .whenPressed(new ExtendCommand(extend));
        operatorOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new HandoffCommand(wrist))
                .whenPressed(new RetractCommand(extend));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new EjectCommand(intake));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new CancelCommand(intake));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new ToggleAlliance(intake));
        operatorOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new CollectSample(intake));
        operatorOp.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new HandoffCommand(wrist));






        pass.setDefaultCommand(new PassCommand(pass,operatorOp::getLeftY));
        intake.rainbowlight();

    }
}
