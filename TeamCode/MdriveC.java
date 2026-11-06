package org.firstinspires.ftc.teamcode; 
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot; 
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
* Mecanum drivetrain TeleOp with optional field-centric driving.
*
* Controls :
*  - Left stick:   translate (forward/back)
*  - Right stick (RL movement) : rotate
*  - Right bumper:  hold for slower mode (precision for pollen?)
*  - A:             toggle field-centric (away from the driver) / robot-centric (the front of the robot)
*  - Back:          reset IMU heading (face the robot "forward" first)
*
* Hardware config names (will need to change this to match our robots config):
*  frontLeft, frontRight, backLeft, backRight, imu
*/

@TeleOp(name = "MdriveC", group = "TeleOp")
public class MdriveC extends LinearOpMode { 

    private DcMotor frontLeft, frontRight, backLeft, backRight; 
    private IMU imu;

    private boolean fieldCentric = true; //robot starts field centric//
    private boolean lastA = false; //lastA = flip once per press//

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        // Typical setup: reverse the left side. if robot drives wrong, flip these.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // IMU: set these to match how the Control Hub is mounted on robot.
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        telemetry.addLine("Ready. Point robot forward, then press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Toggle drive mode on A press (edge detection)
            if (gamepad1.a && !lastA) fieldCentric = !fieldCentric;
            lastA = gamepad1.a;

            if (gamepad1.back) imu.resetYaw();

            double y  = -gamepad1.left_stick_y;  // forward is negative on the stick
            double x  = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (fieldCentric) {
                // Rotate the joystick vector by the negative of the robot heading
                double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
                double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
                x = rotX * 1.1; // counteract imperfect strafing
                y = rotY;
            }

            // Normalize so no wheel exceeds 1.0 while keeping the movement ratios
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double fl = (y + x + rx) / denominator;
            double bl = (y - x + rx) / denominator;
            double fr = (y - x - rx) / denominator;
            double br = (y + x - rx) / denominator;

            double speed = gamepad1.right_bumper ? 0.35 : 1.0;

            frontLeft.setPower(fl * speed);
            backLeft.setPower(bl * speed);
            frontRight.setPower(fr * speed);
            backRight.setPower(br * speed);

            telemetry.addData("Mode", fieldCentric ? "Field-centric" : "Robot-centric");
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
            telemetry.addData("Slow mode", gamepad1.right_bumper);
            telemetry.update();
        }
    }
}
