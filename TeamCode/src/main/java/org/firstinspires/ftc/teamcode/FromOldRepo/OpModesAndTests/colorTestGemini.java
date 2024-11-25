package org.firstinspires.ftc.teamcode.FromOldRepo.OpModesAndTests;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Sensor: Color", group = "Sensor")
@Disabled
public class colorTestGemini extends LinearOpMode {

    ColorSensor colorSensor;
    RevBlinkinLedDriver blinkin;

    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class,



                "blinkin");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            if(colorSensor.red()>colorSensor.blue() && colorSensor.red()>colorSensor.green()) {
                telemetry.addData("Color", "Red");
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);

            } else if(colorSensor.blue()>colorSensor.red() && colorSensor.blue()>colorSensor.green()) {
                telemetry.addData("Color", "Blue");
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else if(colorSensor.red()>colorSensor.blue() && colorSensor.green()>colorSensor.blue()) {
                telemetry.addData("Color", "Yellow");
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else {
                telemetry.addData("Color", "Unknown");
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            }
            telemetry.update();
        }
    }
}