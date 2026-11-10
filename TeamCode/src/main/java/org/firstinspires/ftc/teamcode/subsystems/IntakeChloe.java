package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeChloe {

    // Use 0.3 for the first test, then raise it
    private static final double INTAKE_POWER = 1.0;

    private CRServo servo;

    public void init(HardwareMap hwMap) {
        // "intake" must match the name in your Driver Hub configuration
        servo = hwMap.get(CRServo.class, "intake");

        // KEY IN THE STARTING DIRECTION HERE: FORWARD or REVERSE
        servo.setDirection(DcMotorSimple.Direction.FORWARD);

        servo.setPower(0); // start stopped
    }

    public void in() {
        servo.setPower(INTAKE_POWER);
    }

    public void out() {
        servo.setPower(-INTAKE_POWER);
    }

    public void stop() {
        servo.setPower(0);
    }
}