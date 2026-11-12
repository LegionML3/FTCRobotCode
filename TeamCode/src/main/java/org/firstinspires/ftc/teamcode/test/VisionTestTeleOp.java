package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/*
 * Bench-test OpMode for VisionSubsystem. It does not drive the robot.
 * Point the webcam at an AprilTag and read the numbers on the Driver Station.
 *
 * Controls (gamepad 1):
 *   A : turn AprilTag processing on / off
 *
 * Needs a webcam named "web1" in the Driver Hub configuration.
 */
@TeleOp(name = "Vision Test", group = "Test")
public class VisionTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        VisionSubsystem vision = new VisionSubsystem();
        vision.init(hardwareMap);

        boolean enabled = true;
        boolean lastA = false;

        if (vision.isCameraReady()) {
            telemetry.addData("Status", "Camera ready. Press Play.");
        } else {
            telemetry.addData("Status", "CAMERA FAILED: " + vision.getErrorMessage());
            telemetry.addLine("Check that the webcam is plugged in and named web1 in Configure Robot.");
        }
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // A counts as "pressed" only on the loop where it goes from up to down.
            if (gamepad1.a && !lastA) {
                enabled = !enabled;
                vision.setEnabled(enabled);
            }
            lastA = gamepad1.a;

            telemetry.addData("AprilTag processing", enabled ? "ON" : "OFF");
            vision.addTelemetry(telemetry);

            AprilTagDetection closest = vision.getClosestTag();
            if (closest != null) {
                telemetry.addData("Closest tag ID", VisionSubsystem.getId(closest));
            }
            telemetry.update();
        }

        // Always release the camera when the OpMode ends.
        vision.stop();
    }
}