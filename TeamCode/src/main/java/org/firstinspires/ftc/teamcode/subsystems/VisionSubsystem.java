package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;

/**
 * Webcam + AprilTag vision, wrapped so a missing or unplugged camera can never crash an OpMode.
 *
 * Hardware config name (Driver Hub -> Configure Robot): "web1"  (webcam)
 *
 * Usage:
 *   VisionSubsystem vision = new VisionSubsystem();
 *   vision.init(hardwareMap);
 *   ...
 *   AprilTagDetection tag = vision.getClosestTag();   // null if nothing is seen
 *   ...
 *   vision.stop();   // at the end of the OpMode
 */
public class VisionSubsystem {

    // Must match the webcam name in the Driver Hub configuration EXACTLY.
    private static final String CAMERA_NAME = "web1";

    private AprilTagProcessor aprilTag;
    private VisionPortal portal;
    private boolean cameraReady = false;
    private String errorMessage = "";

    /** Call once in init. If the camera is missing, this does NOT throw; check isCameraReady(). */
    public void init(HardwareMap hwMap) {
        try {
            aprilTag = new AprilTagProcessor.Builder().build();

            portal = new VisionPortal.Builder()
                    .setCamera(hwMap.get(WebcamName.class, CAMERA_NAME))
                    .addProcessor(aprilTag)
                    .build();

            cameraReady = true;
        } catch (Exception e) {
            cameraReady = false;
            errorMessage = String.valueOf(e.getMessage());
        }
    }

    /** True if the camera and AprilTag processor started without an error. */
    public boolean isCameraReady() {
        return cameraReady;
    }

    /** The error from init() if the camera failed to start (empty string if all is well). */
    public String getErrorMessage() {
        return errorMessage;
    }

    /** All tags currently seen. Returns an empty list (never null) if the camera is not ready. */
    public List<AprilTagDetection> getDetections() {
        if (!cameraReady) {
            return new ArrayList<>();
        }
        return aprilTag.getDetections();
    }

    /**
     * The ID number of a detection, or -1 if it is not a single tag (for example a tag cluster).
     * In SDK 12 the ID only exists on AprilTagSingleDetection, so we check the type first.
     */
    public static int getId(AprilTagDetection detection) {
        if (detection instanceof AprilTagSingleDetection) {
            return ((AprilTagSingleDetection) detection).id;
        }
        return -1;
    }

    /** The tag with the given ID, or null if it is not currently visible. */
    public AprilTagDetection getTag(int id) {
        for (AprilTagDetection detection : getDetections()) {
            if (getId(detection) == id) {
                return detection;
            }
        }
        return null;
    }

    /** True if the tag with the given ID is currently visible. */
    public boolean hasTag(int id) {
        return getTag(id) != null;
    }

    /**
     * The nearest tag that has pose data, or null if none.
     * (Tags that are not in the SDK's tag library have no pose, so they are skipped.)
     */
    public AprilTagDetection getClosestTag() {
        AprilTagDetection closest = null;
        for (AprilTagDetection detection : getDetections()) {
            if (detection.ftcPose == null) {
                continue;
            }
            if (closest == null || detection.ftcPose.range < closest.ftcPose.range) {
                closest = detection;
            }
        }
        return closest;
    }

    /** Turn AprilTag processing on or off (off saves CPU when you do not need vision). */
    public void setEnabled(boolean enabled) {
        if (cameraReady) {
            portal.setProcessorEnabled(aprilTag, enabled);
        }
    }

    /** Shows camera state and every visible tag on the Driver Station. */
    public void addTelemetry(Telemetry telemetry) {
        if (!cameraReady) {
            telemetry.addData("Vision", "CAMERA NOT READY: " + errorMessage);
            return;
        }

        List<AprilTagDetection> detections = getDetections();
        telemetry.addData("Vision camera", portal.getCameraState());
        telemetry.addData("Tags seen", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.ftcPose != null) {
                telemetry.addLine(String.format(
                        "ID %d | range %.1f in | bearing %.1f deg | yaw %.1f deg",
                        getId(detection),
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.yaw));
            } else {
                telemetry.addLine(String.format("ID %d | no pose (not in tag library)", getId(detection)));
            }
        }
    }

    /** Call at the end of the OpMode to release the camera. Safe to call more than once. */
    public void stop() {
        if (portal != null) {
            portal.close();
            portal = null;
        }
        cameraReady = false;
    }
}