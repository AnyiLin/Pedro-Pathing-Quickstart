package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem extends SubsystemBase {
    private DcMotor motor;
    private Telemetry telemetry;
    private ColorSensor colorSensor;
    private RevBlinkinLedDriver blinkin;
    private boolean RedAlliance = false;
    private DistanceSensor distanceSensor;
    private ServoImplEx allianceColor;
    private BoxxySubsystem box;

    public IntakeSubsystem(Telemetry telemetry, DcMotor motor, ColorSensor colorSensor, RevBlinkinLedDriver blinkin,
                           DistanceSensor distanceSensor, ServoImplEx allianceColor) {
        this.telemetry = telemetry;
        this.motor = motor;
        this.colorSensor = colorSensor;
        this.blinkin = blinkin;
        this.distanceSensor = distanceSensor;
        this.allianceColor = allianceColor;
        this.box = box;
        this.motor.setDirection(DcMotor.Direction.REVERSE);
        // start as blue alliance
        this.allianceColor.setPosition(0.61);
    }

    @Override
    public void periodic() {
//        telemetry.addData("Motor running", motor.getPower());
//        telemetry.addData("Distance Sensor", distanceSensor.getDistance(DistanceUnit.INCH));
//        telemetry.addData("Red Sensor", colorSensor.red());
//        telemetry.addData("Blue Sensor", colorSensor.blue());
//        telemetry.addData("Green Sensor", colorSensor.green());
//        telemetry.addData("Red Sample", isColorSensorRed());
//        telemetry.addData("Blue Sample", isColorSensorBlue());
//        telemetry.addData("Yellow Sample", isColorSensorYellow());
//
//        if (isColorSensorRed()) {
//            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//        } else if (isColorSensorBlue()) {
//            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
//        } else if (isColorSensorYellow()) {
//            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//        } else {
//            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
//        }
        if (RedAlliance) {
            telemetry.addData("Alliance", "Red");
        } else {
            telemetry.addData("Alliance", "Blue");
        }
//        telemetry.update();
    }
    public void runMotor() {
        motor.setPower(-0.75);
    }
    public void rejectMotor(){motor.setPower(0.75);}
    public void redlight(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }
    public void bluelight(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }
    public void yellowlight(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }
    public void rainbowlight(){
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }

    public void stopMotor() {
        motor.setPower(0);
    }

    public boolean hasSample() { return (distanceSensor.getDistance(DistanceUnit.INCH) < 2.9); }

    public boolean isColorSensorRed() {
        return colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green();
    }

    public boolean isColorSensorBlue() {
        return colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green();
    }

    // Update or remove method as yellow detection isn't used in the intake, only periodic color update
    public boolean isColorSensorYellow() {
        // return colorSensor.red() > colorSensor.blue() && colorSensor.green() > colorSensor.blue();
        return colorSensor.red() > 150 && colorSensor.blue() < 100 && colorSensor.green() > 150;
        // disable for now, as values need adjusting
        //red > 800 & green > 900 & blue < 400
    }

    public void changeAlliance() {
        this.RedAlliance = !getRedAlliance();
        if(RedAlliance) {
            allianceColor.setPosition(0.28);
        } else {
            allianceColor.setPosition(0.61);
        }
    }

    public boolean getRedAlliance() {
        return RedAlliance;
    }
}
