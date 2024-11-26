package org.firstinspires.ftc.teamcode.FromOldRepo.OpModesAndTests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.CameraAdjustTeleCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.TelePedroDriveCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;


@TeleOp(name = "26 Practice")
@Disabled
public class PedroDriveTest extends CommandOpMode {
    private Motor leftFront, rightFront, leftRear, rightRear;

    private Follower follower;

    private PedroDriveSubsystem pedroDriveSubsystem;
    private TelePedroDriveCommand telePedroDriveCommand;
    private CameraAdjustTeleCommand cameraAdjustTeleCommand;

    private GamepadEx driverOp;
    private Button a;

    @Override
    public void initialize(){
        follower = new Follower(hardwareMap);

        leftFront = new Motor(hardwareMap, "frontLeft");
        leftRear = new Motor(hardwareMap, "frontRight");
        rightRear = new Motor(hardwareMap, "backLeft");
        rightFront = new Motor(hardwareMap, "backRight");
        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        driverOp = new GamepadEx(gamepad1);

        follower.startTeleopDrive();
        follower.setMaxPower(1);
        follower.startTeleopDrive();

        pedroDriveSubsystem = new PedroDriveSubsystem( follower);
        telePedroDriveCommand = new TelePedroDriveCommand(pedroDriveSubsystem, telemetry, driverOp::getLeftY, driverOp::getLeftX, driverOp::getRightX, true);
        cameraAdjustTeleCommand = new CameraAdjustTeleCommand(pedroDriveSubsystem);

//        a = (new GamepadButton(driverOp, GamepadKeys.Button.A))
//                .whenPressed(cameraAdjustTeleCommand);
        register(pedroDriveSubsystem, pedroDriveSubsystem);

        pedroDriveSubsystem.setDefaultCommand(telePedroDriveCommand);
    }
}
