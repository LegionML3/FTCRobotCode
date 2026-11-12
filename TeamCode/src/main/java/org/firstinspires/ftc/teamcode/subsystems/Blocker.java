package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A one-servo blocker with two positions: OPEN (out of the way) and BLOCK (in the way).
 * It works the same whether it is a gate that holds Pollen back or an arm that blocks.
 *
 * Hardware config name (Driver Hub -> Configure Robot): "blocker"  (a Servo)
 *
 * IMPORTANT: OPEN_POSITION and BLOCK_POSITION below are placeholders. Find the real values
 * with the "Blocker Test" OpMode (D-pad up/down moves the servo a little at a time), then
 * type the numbers in here. Start with small moves so the servo does not strain against a stop.
 */
public class Blocker {

    // Must match the servo name in the Driver Hub configuration EXACTLY.
    private static final String SERVO_NAME = "blocker";

    // PLACEHOLDERS: tune these with BlockerTestTeleOp.
    public static final double OPEN_POSITION = 0.0;
    public static final double BLOCK_POSITION = 0.5;

    // How close the servo has to be to BLOCK_POSITION to count as "blocking".
    private static final double POSITION_TOLERANCE = 0.01;

    private Servo servo;
    private boolean ready = false;
    private double position = OPEN_POSITION;
    private String errorMessage = "";

    /** Call once in init. Starts OPEN so the blocker is out of the way at the start. */
    public void init(HardwareMap hwMap) {
        try {
            servo = hwMap.get(Servo.class, SERVO_NAME);
            ready = true;
            open();
        } catch (Exception e) {
            ready = false;
            errorMessage = String.valueOf(e.getMessage());
        }
    }

    /** True if the servo was found in the hardware map. */
    public boolean isReady() {
        return ready;
    }

    public String getErrorMessage() {
        return errorMessage;
    }

    public void block() {
        setPosition(BLOCK_POSITION);
    }

    public void open() {
        setPosition(OPEN_POSITION);
    }

    /** Switches between BLOCK and OPEN. */
    public void toggle() {
        if (isBlocking()) {
            open();
        } else {
            block();
        }
    }

    /** True if the servo is at (or very near) the BLOCK position. */
    public boolean isBlocking() {
        return Math.abs(position - BLOCK_POSITION) < POSITION_TOLERANCE;
    }

    /** Moves the servo to any position from 0.0 to 1.0. Used for finding the right positions. */
    public void setPosition(double newPosition) {
        // Keep the value inside the legal servo range.
        position = Math.max(0.0, Math.min(1.0, newPosition));
        if (ready) {
            servo.setPosition(position);
        }
    }

    /** The last position we told the servo to go to. */
    public double getPosition() {
        return position;
    }

    public void addTelemetry(Telemetry telemetry) {
        if (!ready) {
            telemetry.addData("Blocker", "NOT FOUND: " + errorMessage);
            return;
        }
        telemetry.addData("Blocker", isBlocking() ? "BLOCKING" : "OPEN / other");
        telemetry.addData("Blocker position", "%.2f", position);
    }
}