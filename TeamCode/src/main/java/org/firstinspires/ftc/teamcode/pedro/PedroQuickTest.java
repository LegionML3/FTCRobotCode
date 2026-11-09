package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Pedro Quick Test", group = "Pedro")
public class PedroQuickTest extends OpMode {

    private Follower follower;
    private PathChain forward24;
    private boolean lastA = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        forward24 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(0, 0, 0), new Pose(24, 0, 0)))
                .setConstantHeadingInterpolation(0)
                .build();

        telemetry.addLine("A to drive forward 24 in.");
        telemetry.addLine("Press B to stop. Push the robot by hand to see locayion change.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        boolean a = gamepad1.a;
        if (a && !lastA) {
            follower.setStartingPose(new Pose(0, 0, 0)); // reset pose to origin
            follower.followPath(forward24);
        }
        lastA = a;

        if (gamepad1.b) {
            follower.breakFollowing();
        }

        telemetry.addData("x (in)", follower.getPose().getX());
        telemetry.addData("y (in)", follower.getPose().getY());
        telemetry.addData("heading (rad)", follower.getPose().getHeading());
        telemetry.addData("busy", follower.isBusy());
        telemetry.update();
    }
}