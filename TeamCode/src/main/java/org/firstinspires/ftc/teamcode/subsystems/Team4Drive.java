package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "Matthew Teleop", group = "Teleop")
public class Team4Drive extends LinearOpMode {

    //suppress local field warnings
    @SuppressWarnings("FieldCanBeLocal")
    private DcMotor frontLeftDrive;
    @SuppressWarnings("FieldCanBeLocal")
    private DcMotor backLeftDrive;
    @SuppressWarnings("FieldCanBeLocal")
    private DcMotor frontRightDrive;
    @SuppressWarnings("FieldCanBeLocal")
    private DcMotor backRightDrive;
    //imu
    private IMU imu;
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
        //get motors from hardware map
        frontLeftDrive = hardwareMap.get(DcMotor.class, "fl");
        backLeftDrive = hardwareMap.get(DcMotor.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backRightDrive = hardwareMap.get(DcMotor.class, "br");
        //configure motors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        //assign imu for field centric driving
        imu = hardwareMap.get(IMU.class, "IMU");
        //get imu parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        //initialize those parameters
        imu.initialize(parameters);

        telemetry.addLine("Bot Ready rn");
        telemetry.update();

        waitForStart();
        //reset imu
        imu.resetYaw();

        while (opModeIsActive()) {
            //reset imu if back pressed
            if (gamepad1.back){
                imu.resetYaw();
            }
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

            // get robot direction from imu
            double botDirection = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // math stuff: get move direction relative to the robot direction
            double rotX = x * Math.cos(-botDirection) - y * Math.sin(-botDirection);
            double rotY = x * Math.sin(-botDirection) + y * Math.cos(-botDirection);
            //ensure power stays uniform throughout all motors
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

            double flPower = (rotY + rotX + rx) / denominator;
            double blPower = (rotY - rotX + rx) / denominator;
            double frPower = (rotY - rotX - rx) / denominator;
            double brPower = (rotY + rotX - rx) / denominator;

            //powering motors with speed multiplier
            double speed = speedMode.multiplier;
            frontLeftDrive.setPower(flPower * speed);
            backLeftDrive.setPower(blPower * speed);
            frontRightDrive.setPower(frPower * speed);
            backRightDrive.setPower(brPower * speed);

            //display stuff
            telemetry.addData("current speed mode rn: ", "%s (%.0f%%)", speedMode.label, speed * 100);
            telemetry.addData("current direction in degrees: ", "%.1f", Math.toDegrees(botDirection));
            telemetry.addLine("Press D-pad up to change speed");
            telemetry.update();
        }
    }
}