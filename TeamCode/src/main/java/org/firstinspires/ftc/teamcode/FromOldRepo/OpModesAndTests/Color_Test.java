package org.firstinspires.ftc.teamcode.FromOldRepo.OpModesAndTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FromOldRepo.Call_Upon_Classes.Color_Sensor;

@TeleOp
@Disabled
public class Color_Test extends LinearOpMode {
    private final org.firstinspires.ftc.teamcode.FromOldRepo.Call_Upon_Classes.Color_Sensor Color = new Color_Sensor();
    private int redness;
    @Override
    public void runOpMode() throws InterruptedException {
        Color.init_Color(hardwareMap, "color");


        waitForStart();
        while (opModeIsActive()) {Color.get_Telemetry(telemetry);

Color.run_Color();

            telemetry.update();

        }
    }
}
