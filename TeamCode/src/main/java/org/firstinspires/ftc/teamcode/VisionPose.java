package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.List;

@TeleOp(name = "Vision + Pose", group = "TEST")
public class VisionPose extends LinearOpMode {

    // This constant is not used by the processor, but was in your original file.
    // The processor uses the size defined in the .addTag() builder.
    private static final double TAG_SIZE_INCHES = 6.0;

    @Override
    public void runOpMode() throws InterruptedException {

        double fx = 822.317;
        double fy = 822.317;
        double cx = 319.495;
        double cy = 242.502;

        // --- CUSTOM TAG LIBRARY ---
        AprilTagLibrary library = new AprilTagLibrary.Builder()
                .addTag(20, "Blue Target", 6.5, DistanceUnit.INCH)
                .addTag(24, "Red Target", 6.5, DistanceUnit.INCH)// This size (2.76) is used for pose
                .build();

        AprilTagProcessor processor = new AprilTagProcessor.Builder()
                // Set the library to our new custom library
                .setTagLibrary(library)
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)

                .setLensIntrinsics(fx,fy, cx, cy)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor)
                .build();

        telemetry.addLine("Initialized. Press Play to start.");
        telemetry.addLine("Point camera at Tag.");
        telemetry.update();

        waitForStart();

        // -------------------------
        // ASSUMPTIONS (change these)
        // -------------------------
        // 1) Camera height == shooter exit height (in inches). If your camera is mounted higher/lower
        //    than the shooter exit, change cameraToShooterHeightOffset accordingly.
        // 2) The AprilTag is mounted on/near the goal/backboard. The distance from tag center to goal
        //    opening is 9.25 inches VERTICALLY (you gave this).
        // 3) We ignore aerodynamic drag and spin lift. This is an ideal ballistic solution.
        // 4) Flywheel tangential speed = projectile muzzle speed (perfect transfer, no slip).
        // 5) Flywheel radius (inches) should be set to your real flywheel radius.
        //
        // If any of these are wrong for your robot, change the variables below.

        final double cameraToShooterHeightOffset = 0.0; // inches (if camera is 0" above shooter exit)
        final double distTagCenterToGoalEdge = 9.25; // inches (you gave - vertical offset)
        final double flywheelRadiusInches = 2.0; // set your wheel radius in inches (e.g. 2" radius => 4" wheel)
        final double gravityInchesPerSec2 = 386.09; // g in in/s^2

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = processor.getDetections();

            if (!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);
                telemetry.addData("Detected Tag ID", tag.id);

                if (tag.ftcPose != null) {
                    telemetry.addData("Pose", "Available");
                    telemetry.addLine("---");

                    telemetry.addData("X (left/right)", "%.2f in", tag.ftcPose.x);
                    telemetry.addData("Y (forward)", "%.2f in", tag.ftcPose.y);
                    telemetry.addData("Z (up/down)", "%.2f in", tag.ftcPose.z);
                    telemetry.addLine("---");

                    telemetry.addData("Range (hypotenuse)", "%.2f in", (tag.ftcPose.range));
                    telemetry.addData("Bearing (left/right angle)", "%.2f deg", tag.ftcPose.bearing);
                    telemetry.addData("Elevation (up/down angle)", "%.2f deg", tag.ftcPose.elevation);
                    telemetry.addLine("---");

                    telemetry.addData("Yaw", "%.2f deg", tag.ftcPose.yaw);
                    telemetry.addData("Pitch", "%.2f deg", tag.ftcPose.pitch);
                    telemetry.addData("Roll", "%.2f deg", tag.ftcPose.roll);

                    // ======= SHOOTING CALCULATIONS (FIXED) =======

                    // 1. Get horizontal distance (forward) from camera to tag center.
                    // This is our horizontal target distance, 'd'.
                    double d = tag.ftcPose.y; // inches (forward)

                    // 2. Get vertical distance from camera to tag center
                    double verticalToTag = tag.ftcPose.z - cameraToShooterHeightOffset; // inches

                    // 3. Calculate the FINAL target height, 'h'.
                    // The goal is 'distTagCenterToGoalEdge' (9.25 in) ABOVE the tag center.
                    double h = verticalToTag + distTagCenterToGoalEdge;

                    // ---
                    telemetry.addLine("---Shooting calculations (ideal ballistics, no drag)---");
                    telemetry.addData("Target Horiz. Dist (d)", "%.2f in", d);
                    telemetry.addData("Target Vert. Offset (h)", "%.2f in", h);

                    // Define the angles to test
                    double[] anglesDeg = {20.0, 30.0, 40.0, 50.0}; // candidate angles

                    for (double angDeg : anglesDeg) {
                        double theta = Math.toRadians(angDeg);

                        // projectile motion equation:
                        // v^2 = g * d^2 / (2 * cos^2(theta) * (d * tan(theta) - h))
                        // where d = horizontal distance, h = vertical target offset (target - shooter)
                        double cosTheta = Math.cos(theta);
                        double tanTheta = Math.tan(theta);

                        double denom = 2.0 * cosTheta * cosTheta * (d * tanTheta - h);
                        if (denom <= 0.0) {
                            telemetry.addData("Angle " + (int)angDeg + "°", "No solution (denom<=0)");
                            continue;
                        }

                        double v2 = gravityInchesPerSec2 * d * d / denom;
                        if (v2 <= 0.0) {
                            telemetry.addData("Angle " + (int)angDeg + "°", "No solution (v^2<=0)");
                            continue;
                        }

                        double v = Math.sqrt(v2); // muzzle speed in in/s

                        // convert to more human-friendly units:
                        double vFtPerSec = v / 12.0; // ft/s
                        double vMetersPerSec = v * 0.0254; // m/s

                        // Flywheel RPM calculation (single wheel, tangential speed = muzzle speed)
                        double wheelCircumferenceIn = 2.0 * Math.PI * flywheelRadiusInches; // in
                        double wheelRevsPerSec = v / wheelCircumferenceIn; // rev/s
                        double wheelRPM = wheelRevsPerSec * 60.0;

                        telemetry.addData(String.format("Angle %.0f° v", angDeg),
                                "%.1f in/s (%.2f ft/s, %.2f m/s) RPM=%.0f",
                                v, vFtPerSec, vMetersPerSec, wheelRPM);
                    }

                    telemetry.addLine("---Notes---");
                    telemetry.addLine("Assumes no drag, perfect transfer from wheel to ball.");
                    telemetry.addLine("Adjust flywheelRadiusInches & cameraToShooterHeightOffset in code for your robot.");

                } else {
                    telemetry.addData("Pose", "Unavailable");
                }

            } else {
                telemetry.addData("Status", "No tag detected");
            }

            telemetry.update();
            sleep(50);
        }

        portal.close();
    }
}
