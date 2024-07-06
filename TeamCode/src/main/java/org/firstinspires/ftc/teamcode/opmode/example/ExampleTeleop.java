
package org.firstinspires.ftc.teamcode.opmode.example;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.config.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.config.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.config.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.config.subsystem.ClawSubsystem;

/**
 * This is the TeleOpEnhancements OpMode. It is an example usage of the TeleOp enhancements that
 * Pedro Pathing is capable of.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/21/2024
 */
@TeleOp(name = "Example Teleop", group = "Examples")
public class ExampleTeleop extends OpMode {
    private Follower follower;
    private ClawSubsystem claw;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private Vector driveVector;
    private Vector headingVector;

    /** This method is call once when init is played, it initlizes the follower and subsystems **/
    @Override
    public void init() {
        follower = new Follower(hardwareMap, false);
        claw = new ClawSubsystem(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveVector = new Vector();
        headingVector = new Vector();
    }

    /** This method is called continously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
    }

    /** This is the main loop of the Opmode and runs continuously after play **/
    @Override
    public void loop() {

        /** Movement Sector **/
        driveVector.setOrthogonalComponents(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        driveVector.setMagnitude(MathFunctions.clamp(driveVector.getMagnitude(), 0, 1));
        driveVector.rotateVector(follower.getPose().getHeading());
        headingVector.setComponents(-gamepad1.left_stick_x, follower.getPose().getHeading());
        follower.setMovementVectors(new Vector(), headingVector, driveVector);
        follower.update();

        /** Claw Sector **/
        if (gamepad1.left_bumper) {
            claw.closeLClaw();
        } else {
            claw.openLClaw();
        }

        if (gamepad1.right_bumper) {
            claw.closeRClaw();
        } else {
            claw.openRClaw();
        }

        /** This could be paired with a PIDF to set the target position of the lift in teleop.
         * For this, you would have to update the lift pid and make sure to initilize the lift subsystem.
        **/

        /*
        if (gamepad1.left_trigger > 0.5) {
            lift.setTarget(lTarget-50);
        }

        if (gamepad1.right_trigger > 0.5) {
            lift.setTarget(lTarget+50);
        }
        */
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}
