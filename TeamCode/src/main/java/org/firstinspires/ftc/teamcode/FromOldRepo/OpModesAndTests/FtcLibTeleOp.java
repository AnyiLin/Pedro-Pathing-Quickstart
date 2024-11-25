package org.firstinspires.ftc.teamcode.FromOldRepo.OpModesAndTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FromOldRepo.Examples.GobuildaSample.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.CameraAdjustTeleCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.ExtensionOutCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.ImuCommands.ImuResetCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.LocalizerCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.TelePedroDriveCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.TelemetryCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands.VisionCommands.LimelightAprilTagCommand;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.AutoDriveSubsystem;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.LocalizerSubsystem;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.TelemetrySubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;


@TeleOp(name = "FtcLibTeleOp")
@Disabled
public class FtcLibTeleOp extends CommandOpMode {
    private Motor leftFront, rightFront, leftRear, rightRear;
    private DcMotorEx extensionMotor;

    private Follower follower;
    private PedroDriveSubsystem pedroDriveSubsystem;
    private TelePedroDriveCommand telePedroDriveCommand;
    private CameraAdjustTeleCommand cameraAdjustTeleCommand;

    private Limelight3A limelight;
    private LimelightSubsystem limelightSubsystem;

    private LocalizerSubsystem localizerSubsystem;

    private TelemetrySubsystem telemetrySubsystem;
    private TelemetryCommand telemetryCommand;

    private OdometrySubsystem odometrySubsystem;
    private GoBildaPinpointDriver odometry;
    private Telemetry mtelemetry;

    private ExtensionSubsystem extensionSubsystem;
    private ExtensionOutCommand extensionOutCommand;

    private GamepadEx driverOp, operatorOp;
    private Button aButton, bButton;

    @Override
    public void initialize(){
        mtelemetry = new  MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        leftFront = new Motor(hardwareMap, "frontLeft");
        leftRear = new Motor(hardwareMap, "frontRight");
        rightRear = new Motor(hardwareMap, "backLeft");
        rightFront = new Motor(hardwareMap, "backRight");
        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        driverOp = new GamepadEx(gamepad1);
        operatorOp = new GamepadEx(gamepad2);

        follower.startTeleopDrive();
        follower.setMaxPower(1);
        follower.startTeleopDrive();

        pedroDriveSubsystem = new PedroDriveSubsystem( follower);
        telePedroDriveCommand = new TelePedroDriveCommand(pedroDriveSubsystem, telemetry, driverOp::getLeftY, driverOp::getLeftX, driverOp::getRightX, true);

        extensionMotor = hardwareMap.get(DcMotorEx.class, "extensionMotor");
        extensionSubsystem = new ExtensionSubsystem(extensionMotor);
        extensionOutCommand = new ExtensionOutCommand(extensionSubsystem);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelightSubsystem = new LimelightSubsystem(limelight, telemetry);

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");
        odometrySubsystem = new OdometrySubsystem(odometry,telemetry, new Pose2D(DistanceUnit.INCH, 0, 0 , AngleUnit.RADIANS, 0));

        localizerSubsystem = new LocalizerSubsystem(telemetry, limelightSubsystem, odometrySubsystem);

//        telemetrySubsystem = new TelemetrySubsystem(mtelemetry, limelightSubsystem, localizerSubsystem, odometrySubsystem);
//        telemetryCommand= new TelemetryCommand(telemetrySubsystem);

        bButton = (new GamepadButton(driverOp, GamepadKeys.Button.B))
                .whenPressed(extensionOutCommand);
        cameraAdjustTeleCommand = new CameraAdjustTeleCommand(pedroDriveSubsystem);

        aButton = (new GamepadButton(driverOp, GamepadKeys.Button.A))
                .whenPressed(cameraAdjustTeleCommand);

        register(limelightSubsystem, localizerSubsystem, odometrySubsystem, extensionSubsystem, pedroDriveSubsystem);

//        telemetrySubsystem.setDefaultCommand(telemetryCommand);
        pedroDriveSubsystem.setDefaultCommand(telePedroDriveCommand);
    }
}
