package org.firstinspires.ftc.teamcode.pedroPathing.localization.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;

/**
 * This is the LocalizerDirection OpMode. This allows you to check if the encoders
 * used in your localizer have the correct direction.
 *
 * @author Julien Vanier - 21615 Rocket Robotics
 * @version 1.0, 5/6/2024
 */
@Config
@TeleOp(group = "Pedro Pathing Tuning", name = "Localizer Direction")
public class LocalizerDirection extends OpMode {
    private PoseUpdater poseUpdater;
    private Telemetry telemetryA;

    /**
     * This initializes the PoseUpdater and the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        poseUpdater = new PoseUpdater(hardwareMap);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will print the encoder ticks on telemetry.");
        telemetryA.addLine("Forward encoders should increase when pushing the robot forward.");
        telemetryA.addLine("Strafe encoder should increase when pushing the robot right.");
        telemetryA.addLine("Yaw should go from 0 to 3.14 when the robot does half a turn counter-clockwise.");
        telemetryA.update();
    }

    /**
     * This updates the robot's pose estimate and send telemetry.
     */
    @Override
    public void loop() {
        poseUpdater.update();

        poseUpdater.debug(telemetryA);
        telemetryA.update();
    }
}
