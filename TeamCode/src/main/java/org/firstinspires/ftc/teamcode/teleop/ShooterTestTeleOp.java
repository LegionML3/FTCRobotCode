package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Bench-test OpMode for the Shooter class. Use it to tune RPM and timing BEFORE putting
 * the shooter into your real TeleOp. It does not drive the robot.
 *
 * Controls (gamepad 1):
 *   Y          : turn the flywheel on / off
 *   A          : fire 1 ball
 *   B          : fire a burst of 3 balls
 *   X          : cancel queued shots (flywheel keeps spinning)
 *   D-pad up   : target RPM +100
 *   D-pad down : target RPM -100
 *   Back       : emergency stop (flywheel off, shots cleared)
 */
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        Shooter shooter = new Shooter(hardwareMap);

        // Remember last loop's button states so a button HELD DOWN only triggers once.
        boolean lastY = false, lastA = false, lastB = false, lastUp = false, lastDown = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // A button counts as "pressed" only on the loop where it goes from up to down.
            if (gamepad1.y && !lastY)          shooter.toggleSpin();
            if (gamepad1.a && !lastA)          shooter.fire(1);
            if (gamepad1.b && !lastB)          shooter.fire(3);
            if (gamepad1.dpad_up && !lastUp)   shooter.adjustTargetRpm(100);
            if (gamepad1.dpad_down && !lastDown) shooter.adjustTargetRpm(-100);
            if (gamepad1.x)                    shooter.cancelShots();
            if (gamepad1.back)                 shooter.stop();

            lastY = gamepad1.y;
            lastA = gamepad1.a;
            lastB = gamepad1.b;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;

            // The shooter only moves when update() runs, so this must be called every loop.
            shooter.update();

            shooter.addTelemetry(telemetry);
            telemetry.update();
        }

        // Make sure everything is off when the OpMode ends.
        shooter.stop();
    }
}
