package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeChloe;

@TeleOp(name = "Intake TeleOp")
public class IntakeTeleOpChloe extends LinearOpMode {

    @Override
    public void runOpMode() {
        IntakeChloe intake = new IntakeChloe();
        intake.init(hardwareMap);

        telemetry.addLine("Ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                intake.in();
                telemetry.addData("Intake", "IN");
            } else if (gamepad1.left_bumper) {
                intake.out();
                telemetry.addData("Intake", "OUT");
            } else {
                intake.stop();
                telemetry.addData("Intake", "STOPPED");
            }
            telemetry.update();
        }
    }
}