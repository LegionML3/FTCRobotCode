package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "Matthew Teleop", group = "Teleop")
public class Team4Drive extends LinearOpMode {
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    // Speed modes: press D-pad up to cycle speed modes
    private enum SpeedMode {
        slow("Slow", 0.25),
        medium("Normal", 0.5),
        fast("Fast", 1.0);

        final String label;
        final double multiplier;

        SpeedMode(String label, double multiplier) {
            this.label = label;
            this.multiplier = multiplier;
        }

        SpeedMode next() {
            SpeedMode[] all = values();
            return all[(ordinal() + 1) % all.length];
        }
    }

    //instantiate the object from speed mode class
    private SpeedMode speedMode = SpeedMode.medium;
    private boolean lastKeyPressed = false; // used to detect a new button press

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Bot Ready rn");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //change speed mode
            boolean keyPressed = gamepad1.dpad_up;
            if (keyPressed && !lastKeyPressed) {
                speedMode = speedMode.next();
            }
            lastKeyPressed = keyPressed;

            // drive math stuff
            double y = -gamepad1.left_stick_y;   // forward/back (stick up is negative fyi)
            double x = gamepad1.left_stick_x;    // strafe
            double rx = gamepad1.right_stick_x;  // turn

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double flPower = (y + x + rx) / denominator;
            double blPower = (y - x + rx) / denominator;
            double frPower = (y - x - rx) / denominator;
            double brPower = (y + x - rx) / denominator;

            //powering motors
            double speed = speedMode.multiplier;
            frontLeftDrive.setPower(flPower * speed);
            backLeftDrive.setPower(blPower * speed);
            frontRightDrive.setPower(frPower * speed);
            backRightDrive.setPower(brPower * speed);

            //display stuff
            telemetry.addData("Speed mode", "%s (%.0f%%)", speedMode.label, speed * 100);
            telemetry.addLine("Press D-pad up to change speed");
            telemetry.update();
        }
    }
}