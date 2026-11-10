/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// The package must match the folder this file lives in (TeamCode/.../teamcode).
package org.firstinspires.ftc.teamcode;

// Imports: these pull in the FTC classes we use below.
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // base class for a "linear" OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;       // marks this as a driver-controlled OpMode
import com.qualcomm.robotcore.hardware.DcMotor;              // represents one drive motor
import com.qualcomm.robotcore.util.ElapsedTime;              // a simple stopwatch

/*
 * Mecanum (holonomic) TeleOp with a "slow mode".
 *
 * Controls (gamepad 1):
 *   Left stick   : drive forward/backward and strafe left/right
 *   Right stick  : rotate the robot
 *   Right bumper : HOLD to cut the robot's speed in half (precision mode).
 *                  Let go to return to full speed.
 */

// @TeleOp makes this OpMode show up in the TeleOp list on the Driver Station.
// "name" is what you see in the menu; "group" is just for sorting the list.
@TeleOp(name="Basic: Test TeleOp", group="Linear OpMode")
public class TestTeleop extends LinearOpMode {

    // ---------------------------------------------------------------------------------------
    // Settings you may want to tweak
    // ---------------------------------------------------------------------------------------

    // Multiplier applied to all wheel power while slow mode is active.
    // 0.5 = half speed. Change this number to make slow mode faster or slower.
    private static final double SLOW_MODE_MULTIPLIER = 0.5;

    // Multiplier used when slow mode is NOT active (1.0 = full speed).
    private static final double FULL_SPEED_MULTIPLIER = 1.0;

    // ---------------------------------------------------------------------------------------
    // Robot hardware and helper objects
    // ---------------------------------------------------------------------------------------

    // Stopwatch used to show how long the OpMode has been running.
    private final ElapsedTime runtime = new ElapsedTime();

    // One variable per drive motor. They start as null and get filled in inside runOpMode().
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    // runOpMode() is the main method. The FTC app calls it when you press INIT.
    @Override
    public void runOpMode() {

        // -----------------------------------------------------------------------------------
        // 1) Hardware setup
        // -----------------------------------------------------------------------------------

        // Look up each motor by the name you gave it in "Configure Robot" on the Driver Hub.
        // These strings must match that configuration EXACTLY (including capitalization),
        // or the OpMode will crash at INIT with a "hardware not found" error.
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "fl");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "bl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "fr");
        backRightDrive  = hardwareMap.get(DcMotor.class, "br");

        // Motors on opposite sides of the robot face opposite directions, so one side has to
        // be reversed for "positive power" to mean "drive forward" on every wheel.
        // If a wheel spins the wrong way during testing, flip FORWARD <-> REVERSE for that motor.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver everything is ready. This shows on the Driver Station screen.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Pause here until the driver presses the PLAY button on the Driver Station.
        waitForStart();

        // Restart the stopwatch so it counts from the moment PLAY was pressed.
        runtime.reset();

        // -----------------------------------------------------------------------------------
        // 2) Main control loop (repeats until the driver presses STOP)
        // -----------------------------------------------------------------------------------
        while (opModeIsActive()) {

            // --- Read the joysticks -------------------------------------------------------

            // Pushing a stick forward gives a NEGATIVE y value, so we flip the sign (the "-")
            // to make "forward" a positive number.
            double axial   = -gamepad1.left_stick_y;  // forward / backward
            double lateral =  gamepad1.left_stick_x;  // strafe left / right
            double yaw     =  gamepad1.right_stick_x; // rotate left / right

            // --- Mecanum math -------------------------------------------------------------

            // Each wheel gets a different mix of the three motions. This pattern is what lets
            // a mecanum robot drive and strafe in any direction while turning at the same time.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // --- Normalize ----------------------------------------------------------------

            // Motor power can only be between -1.0 and 1.0, but adding the three inputs can
            // produce numbers bigger than 1 (for example, driving forward and strafing at once).
            // Find the largest power requested by any wheel...
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            // ...and if it is over 1.0, divide every wheel by it. This keeps all the wheels in
            // the correct proportion so the robot still moves in the direction you intended.
            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // --- Slow mode ----------------------------------------------------------------

            // While the right bumper is held, use the slow multiplier; otherwise use full speed.
            // (This is a shorthand if/else: condition ? valueIfTrue : valueIfFalse)
            boolean slowMode = gamepad1.right_bumper;
            double speedMultiplier = slowMode ? SLOW_MODE_MULTIPLIER : FULL_SPEED_MULTIPLIER;

            // Scale every wheel by the same amount. Because they all shrink together, the robot
            // still drives in the same direction, just at a reduced speed.
            // This is done AFTER normalizing so the half-speed cap is applied to the final values.
            frontLeftPower  *= speedMultiplier;
            frontRightPower *= speedMultiplier;
            backLeftPower   *= speedMultiplier;
            backRightPower  *= speedMultiplier;

            // --- Motor direction test (disabled) ------------------------------------------

            // To find out which motor is which and whether each one spins the right way,
            // temporarily remove the /* and */ below. Each button then runs ONE motor forward:
            //   X = front left, A = back left, Y = front right, B = back right.
            // Fix any wrong directions using setDirection() above, then comment this out again.
            // Do this with the robot's wheels OFF the ground.
            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;
            frontRightPower = gamepad1.y ? 1.0 : 0.0;
            backRightPower  = gamepad1.b ? 1.0 : 0.0;
            */

            // --- Send power to the motors -------------------------------------------------

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // --- Telemetry (info shown on the Driver Station) -----------------------------

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed mode", slowMode ? "SLOW (half speed)" : "FULL");
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}