package org.firstinspires.ftc.teamcode.FromOldRepo.commandBased.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FromOldRepo.Examples.GobuildaSample.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public class AutoSetStartCommand extends CommandBase {
    private Pose startPose;
    private Follower follower;

    public AutoSetStartCommand(Pose startPose, Follower follower) {
        this.startPose = startPose;
        this.follower = follower;
    }
    @Override
    public void execute() {
        follower.setPose(startPose);
    }
    @Override
    public boolean isFinished() {
//        if (follower.deviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
//            return true;
//        } else {
//            return false;
//        }
        return true;
    }


}
