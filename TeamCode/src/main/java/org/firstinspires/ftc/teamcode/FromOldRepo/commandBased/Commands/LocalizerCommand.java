package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands;


import static java.lang.Math.PI;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.ImuSubsystem;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.LocalizerSubsystem;
import org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Subsystems.OdometrySubsystem;

public class LocalizerCommand extends CommandBase {
    private LocalizerSubsystem localizerSubsystem;
    private OdometrySubsystem odometrySubsystem;
    private LimelightSubsystem limelightSubsystem;

    public LocalizerCommand(LocalizerSubsystem localizerSubsystem, LimelightSubsystem limelightSubsystem, OdometrySubsystem odometrySubsystem){
        this.localizerSubsystem = localizerSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        addRequirements(localizerSubsystem, odometrySubsystem, limelightSubsystem);
    }
    @Override
    public void execute(){
        odometrySubsystem.setOdoPos(new Pose2D(DistanceUnit.INCH, 20, 20, AngleUnit.RADIANS, PI/2));
    }
}
