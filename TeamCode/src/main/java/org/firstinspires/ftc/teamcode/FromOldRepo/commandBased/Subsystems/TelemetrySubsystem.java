package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetrySubsystem extends SubsystemBase {
    private Telemetry telemetry;
    private LimelightSubsystem limelightSubsystem;
    private LocalizerSubsystem localizerSubsystem;
    private OdometrySubsystem odometrySubsystem;

    public TelemetrySubsystem(Telemetry telemetry, LimelightSubsystem limelightSubsystem, LocalizerSubsystem localizerSubsystem, OdometrySubsystem odometrySubsystem){
        this.telemetry = telemetry;
        this.limelightSubsystem = limelightSubsystem;
        this.localizerSubsystem = localizerSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        telemetry.setMsTransmissionInterval(20);

    }
    public void getTelemetry(){
        clearTelemetry();
        localizerSubsystem.getLocalizerTelemetry();
        limelightSubsystem.getLimelightTelemetry();
        odometrySubsystem.getTelemetryOdo();
        updateTelemetry();
    }
    public void updateTelemetry(){
        telemetry.update();
    }
    public void clearTelemetry(){
        telemetry.clearAll();
    }
}
