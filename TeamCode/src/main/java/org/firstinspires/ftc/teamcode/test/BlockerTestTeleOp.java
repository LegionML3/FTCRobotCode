package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Blocker;

/*
 * Bench-test OpMode for the Blocker servo. Use it to find the OPEN and BLOCK positions,
 * then copy those numbers into Blocker.java. It does not drive the robot.
 *
 * Controls (gamepad 1):
 *   A          : go to BLOCK position
 *   B          : go to OPEN position
 *   X          : toggle between BLOCK and OPEN
 *   D-pad up   : move servo +0.02 (to find a new position)
 *   D-pad down : move servo -0.02
 *
 * The current position is shown on the Driver Station. When the blocker is where you want it,
 * write that number down.
 */
@TeleOp(name = "Blocker Test", group = "Test")
public class BlockerTestTeleOp extends LinearOpMode {

    private static final double NUDGE = 0.02;

    @Override
    public void runOpMode() {
        Blocker blocker = new Blocker();
        blocker.init(hardwareMap);

        // Remember last loop's button states so a button HELD DOWN only triggers once.
        boolean lastA = false, lastB = false, lastX = false, lastUp = false, lastDown = false;

        telemetry.addData("Status", blocker.isReady() ? "Initialized" : "SERVO 'blocker' NOT FOUND");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a && !lastA) blocker.block();
            if (gamepad1.b && !lastB) blocker.open();
            if (gamepad1.x && !lastX) blocker.toggle();
            if (gamepad1.dpad_up && !lastUp) blocker.setPosition(blocker.getPosition() + NUDGE);
            if (gamepad1.dpad_down && !lastDown) blocker.setPosition(blocker.getPosition() - NUDGE);

            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastX = gamepad1.x;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            blocker.addTelemetry(telemetry);
            telemetry.update();
        }
    }
}