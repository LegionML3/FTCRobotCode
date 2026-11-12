package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * Simplest possible Pedro Pathing autonomous: drive straight forward and stop.
 * Use it to prove that Pedro and the Pinpoint odometry work before building real autos.
 *
 * Safety net: if the path is still running after TIMEOUT_SECONDS (for example the robot is
 * blocked or the odometry is wrong), the path is cancelled so the robot does not push forever.
 *
 * Start the robot with its front facing the direction you want to drive.
 * Before relying on this, tune the numbers in pedro/Constants.java (especially robot mass).
 */
@Autonomous(name = "Pedro Safety Auto", group = "Auto")
public class PedroSafetyAuto extends OpMode {

    // How far to drive forward, in inches.
    private static final double DRIVE_INCHES = 24.0;

    // Give up and stop if the path has not finished after this many seconds.
    private static final double TIMEOUT_SECONDS = 6.0;

    private Follower follower;
    private PathChain forwardPath;
    private final ElapsedTime timer = new ElapsedTime();

    private boolean finished = false;
    private String finishReason = "running";

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // The robot starts at the origin facing forward (heading 0).
        Pose startPose = new Pose(0, 0, 0);
        follower.setStartingPose(startPose);

        forwardPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, new Pose(DRIVE_INCHES, 0, 0)))
                .setConstantHeadingInterpolation(0)
                .build();

        telemetry.addLine("Ready. Press Play to drive forward " + DRIVE_INCHES + " in.");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
        follower.followPath(forwardPath);
    }

    @Override
    public void loop() {
        // Pedro only moves the robot when update() runs, so this must be called every loop.
        follower.update();

        if (!finished) {
            if (!follower.isBusy()) {
                finished = true;
                finishReason = "path complete";
            } else if (timer.seconds() > TIMEOUT_SECONDS) {
                follower.breakFollowing();
                finished = true;
                finishReason = "TIMEOUT - path cancelled";
            }
        }

        telemetry.addData("status", finishReason);
        telemetry.addData("x (in)", follower.getPose().getX());
        telemetry.addData("y (in)", follower.getPose().getY());
        telemetry.addData("heading (rad)", follower.getPose().getHeading());
        telemetry.addData("time (s)", "%.1f", timer.seconds());
        telemetry.update();
    }
}