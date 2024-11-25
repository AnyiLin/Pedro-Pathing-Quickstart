package org.firstinspires.ftc.teamcode.FromOldRepo.OpModesAndTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FromOldRepo.Call_Upon_Classes.Imu;
import org.firstinspires.ftc.teamcode.FromOldRepo.Call_Upon_Classes.LimeLight;

@TeleOp
@Disabled
public class TestOpMode1 extends LinearOpMode {
    public final Imu imu = new Imu();
    public final LimeLight limeLight = new LimeLight();


    @Override
    public void runOpMode() throws InterruptedException {
        imu.init(hardwareMap);
        limeLight.init(hardwareMap, telemetry);
        telemetry.setMsTransmissionInterval(11);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            limeLight.readAprilTagToTelemetry(telemetry);
            telemetry.addData("Yaw", imu.getImuYawDeg());
            telemetry.update();
        }
    }
}
