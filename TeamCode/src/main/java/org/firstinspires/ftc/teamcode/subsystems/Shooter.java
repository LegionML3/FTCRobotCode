package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Flywheel shooter with a servo "kicker" that pushes one ball into the wheel at a time.
 *
 * HOW IT STAYS FAST AND ACCURATE
 *  - The flywheel runs in closed-loop velocity mode, so it holds the same RPM regardless of load.
 *  - A ball is only fed when the wheel has been inside the RPM tolerance for STABLE_MS.
 *    Every ball therefore leaves at (nearly) the same speed, which keeps shots consistent.
 *  - Between balls the code waits for the wheel to recover, and not a millisecond longer.
 *    The kicker retracts WHILE the wheel recovers, so the two delays overlap.
 *  - The wheel stays spinning between bursts, so the first ball of the next burst has no spin-up delay.
 *
 * HOW TO USE
 *   Shooter shooter = new Shooter(hardwareMap);   // in init
 *   shooter.fire(3);                               // queue 3 shots (also starts the wheel if needed)
 *   shooter.update();                              // call EVERY loop iteration, never skip it
 *
 * Every number marked TUNE below is a starting guess. Change them on your robot.
 */
public class Shooter {

    /** What the shooter is doing right now. */
    public enum State { IDLE, SPINNING_UP, READY, FEEDING, RETRACTING }

    // ---------------------------------------------------------------------------------------
    // Hardware names (must match Configure Robot exactly)
    // ---------------------------------------------------------------------------------------
    private static final String FLYWHEEL_NAME   = "flywheel";
    private static final String FLYWHEEL_2_NAME = "flywheel2"; // optional second motor, ignored if not configured
    private static final String KICKER_NAME     = "kicker";

    // Flip these if a wheel spins the wrong way. A second wheel on the opposite side usually needs REVERSE.
    private static final DcMotorSimple.Direction FLYWHEEL_DIRECTION   = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction FLYWHEEL_2_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // ---------------------------------------------------------------------------------------
    // Motor math (TUNE: use your motor's numbers)
    // ---------------------------------------------------------------------------------------
    // Encoder ticks per rotation of the MOTOR SHAFT. goBILDA 5203 series bare motor = 28.
    // If a gear ratio sits between the motor and the wheel, RPM below is MOTOR rpm, not wheel rpm.
    private static final double TICKS_PER_REV = 28.0;
    // Free speed of the motor (6000 for goBILDA 6000rpm motors). Used to compute the feedforward term.
    private static final double MAX_RPM = 6000.0;

    // Velocity PIDF. F does most of the work; start with P small and raise it if the wheel is slow to settle.
    private static final double P = 12.0;
    private static final double I = 0.0;
    private static final double D = 0.0;
    private static final double F = 32767.0 / (MAX_RPM / 60.0 * TICKS_PER_REV);

    // ---------------------------------------------------------------------------------------
    // Shot timing (TUNE: lower the times until balls start jamming, then back off a little)
    // ---------------------------------------------------------------------------------------
    private static final double KICKER_REST = 0.00; // servo position with the kicker pulled back
    private static final double KICKER_FEED = 0.40; // servo position that pushes a ball into the wheel
    private static final double FEED_MS     = 120;  // time to let the kicker push the ball through
    private static final double RETRACT_MS  = 120;  // time to let the kicker return before the next ball

    // ---------------------------------------------------------------------------------------
    // Accuracy vs. speed (TUNE)
    // ---------------------------------------------------------------------------------------
    // Smaller tolerance = more consistent shots but more waiting between balls.
    private static final double TOLERANCE_RPM = 50.0;
    // How long the wheel must stay inside the tolerance before a ball is fed.
    private static final double STABLE_MS = 30.0;

    private static final double DEFAULT_TARGET_RPM = 3000.0; // TUNE

    // ---------------------------------------------------------------------------------------
    // Internals
    // ---------------------------------------------------------------------------------------
    private final DcMotorEx flywheel;
    private final DcMotorEx flywheel2; // null if not configured
    private final Servo kicker;

    private final ElapsedTime phaseTimer  = new ElapsedTime(); // times FEEDING and RETRACTING
    private final ElapsedTime stableTimer = new ElapsedTime(); // time since the wheel last left the tolerance band

    private State state = State.IDLE;
    private double targetRpm = DEFAULT_TARGET_RPM;
    private boolean flywheelOn = false;
    private int shotsRemaining = 0;

    public Shooter(HardwareMap hardwareMap) {
        flywheel  = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheel2 = hardwareMap.tryGet(DcMotorEx.class, FLYWHEEL_2_NAME); // null if not on the robot
        kicker    = hardwareMap.get(Servo.class, KICKER_NAME);

        setupMotor(flywheel, FLYWHEEL_DIRECTION);
        if (flywheel2 != null) {
            setupMotor(flywheel2, FLYWHEEL_2_DIRECTION);
        }

        kicker.setPosition(KICKER_REST);
    }

    private void setupMotor(DcMotorEx motor, DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);       // enables closed-loop velocity control
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // let the wheel coast down, easier on the gears
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
    }

    // ---------------------------------------------------------------------------------------
    // Commands (call these from your OpMode)
    // ---------------------------------------------------------------------------------------

    /** Start spinning the flywheel and keep it spinning until stop() or toggleSpin(). */
    public void spinUp() {
        flywheelOn = true;
    }

    /** Switch the flywheel on or off. Turning it off also clears any queued shots. */
    public void toggleSpin() {
        if (flywheelOn) {
            stop();
        } else {
            spinUp();
        }
    }

    /**
     * Queue n shots. Each ball is fed as soon as the wheel is at speed. The wheel is switched
     * on automatically and stays on afterwards so the next burst starts instantly.
     */
    public void fire(int n) {
        if (n <= 0) return;
        shotsRemaining += n;
        flywheelOn = true;
    }

    /** Cancel queued shots and pull the kicker back, but leave the flywheel spinning. */
    public void cancelShots() {
        shotsRemaining = 0;
        kicker.setPosition(KICKER_REST);
        state = flywheelOn ? State.SPINNING_UP : State.IDLE;
    }

    /** Stop everything: no more shots, flywheel off, kicker retracted. */
    public void stop() {
        flywheelOn = false;
        shotsRemaining = 0;
        kicker.setPosition(KICKER_REST);
        state = State.IDLE;
        flywheel.setVelocity(0);
        if (flywheel2 != null) flywheel2.setVelocity(0);
    }

    /** Change the target speed (for tuning at practice). Clamped to 0..MAX_RPM. */
    public void adjustTargetRpm(double deltaRpm) {
        targetRpm = Math.max(0, Math.min(MAX_RPM, targetRpm + deltaRpm));
    }

    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0, Math.min(MAX_RPM, rpm));
    }

    // ---------------------------------------------------------------------------------------
    // The main loop. Call once per OpMode loop iteration.
    // ---------------------------------------------------------------------------------------
    public void update() {
        // 1) Tell the motors what speed we want (0 when the flywheel is off).
        double ticksPerSec = rpmToTicksPerSec(flywheelOn ? targetRpm : 0);
        flywheel.setVelocity(ticksPerSec);
        if (flywheel2 != null) flywheel2.setVelocity(ticksPerSec);

        // 2) Decide whether the wheel is "at speed". It has to stay inside the tolerance band
        //    continuously for STABLE_MS; any time it leaves the band the stability clock restarts.
        double errorRpm = Math.abs(getRpm() - targetRpm);
        if (!flywheelOn || errorRpm > TOLERANCE_RPM) {
            stableTimer.reset();
        }
        boolean atSpeed = flywheelOn && stableTimer.milliseconds() >= STABLE_MS;

        // 3) Run the shot state machine.
        switch (state) {
            case IDLE:
                if (flywheelOn) state = State.SPINNING_UP;
                break;

            case SPINNING_UP:
            case READY:
                if (!flywheelOn) {
                    state = State.IDLE;
                    break;
                }
                state = atSpeed ? State.READY : State.SPINNING_UP;

                // At speed with a shot waiting: push a ball into the wheel.
                if (state == State.READY && shotsRemaining > 0) {
                    kicker.setPosition(KICKER_FEED);
                    phaseTimer.reset();
                    state = State.FEEDING;
                }
                break;

            case FEEDING:
                // Give the kicker time to push the ball through, then pull it back.
                if (phaseTimer.milliseconds() >= FEED_MS) {
                    kicker.setPosition(KICKER_REST);
                    phaseTimer.reset();
                    state = State.RETRACTING;
                }
                break;

            case RETRACTING:
                // The wheel is recovering from the shot while the kicker returns. Once the kicker
                // is back, count the shot and go wait for the wheel (usually already recovered).
                if (phaseTimer.milliseconds() >= RETRACT_MS) {
                    shotsRemaining = Math.max(0, shotsRemaining - 1);
                    state = flywheelOn ? State.SPINNING_UP : State.IDLE;
                }
                break;
        }
    }

    // ---------------------------------------------------------------------------------------
    // Status
    // ---------------------------------------------------------------------------------------

    public State getState()        { return state; }
    public boolean isSpinning()    { return flywheelOn; }
    public boolean isReady()       { return state == State.READY; }
    public boolean isBusy()        { return shotsRemaining > 0 || state == State.FEEDING || state == State.RETRACTING; }
    public int getShotsRemaining() { return shotsRemaining; }
    public double getTargetRpm()   { return targetRpm; }

    /** Current flywheel speed in RPM (of the motor shaft). */
    public double getRpm() {
        return flywheel.getVelocity() / TICKS_PER_REV * 60.0;
    }

    /** Puts the useful numbers on the Driver Station screen. Call telemetry.update() yourself afterwards. */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter state", state);
        telemetry.addData("Flywheel on", flywheelOn);
        telemetry.addData("RPM (now / target)", "%.0f / %.0f", getRpm(), targetRpm);
        telemetry.addData("Shots remaining", shotsRemaining);
    }

    private static double rpmToTicksPerSec(double rpm) {
        return rpm / 60.0 * TICKS_PER_REV;
    }
}
