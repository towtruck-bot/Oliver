package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.List;

@Config
public class LLBlockDetectionPostProcessor {
    public enum Block {
        YELLOW(0),
        RED(1),
        BLUE(2);

        private final int pipelineIndex;

        Block(int pipelineIndex) {
            this.pipelineIndex = pipelineIndex;
        }
    }

    public static boolean useExpected = false;
    private Limelight3A ll;
    private Block block;
    private Pose2d blockPos; // Robot centric vaue
    private Pose2d lastPosition;
    private Robot robot;
    private boolean detecting = false;
    private Vector2 offset = new Vector2(0, 0);
    private double lowPassError = 0;
    public static int pollRate = 100;
    private int sumDetections = 0;
    public static int maxAngX = 15;
    public static int maxAngY = 15;
    private double lastOrientation = Double.MAX_VALUE;
    private double orientation = Double.MAX_VALUE;
    private double errorX = 0, errorY = 0, errorHeading = 0;

    public LLBlockDetectionPostProcessor(Robot robot) {
        ll = robot.hardwareMap.get(Limelight3A.class, "limelight");
        block = Block.YELLOW;
        ll.setPollRateHz(pollRate);
        blockPos = new Pose2d(0, 0, 0);
        lastPosition = null;
        this.robot = robot;
    }

    public void start() {
        ll.pipelineSwitch(block.pipelineIndex);
        ll.start();
    }

    public void stop() {
        ll.stop();
    }

    public void setBlockColor(Block block) {
        this.block = block;
        ll.pipelineSwitch(block.pipelineIndex);
    }

    /**
     * <b>IMPORTANT</b>: Update the localizer before<br>
     * If you didn't do this you're stupid btw
     */
    public void update() {
        if (!detecting)
            return;

        LLResult result = ll.getLatestResult();

        // Get robot based pose delta
        Pose2d p = robot.sensors.getOdometryPosition();
        Pose2d pDelta = new Pose2d( // Pose delta
            p.x - lastPosition.x,
            p.y - lastPosition.y,
            p.heading - lastPosition.heading
        );
        Pose2d pDeltaRelative = new Pose2d(
            pDelta.x * Math.cos(p.heading) + pDelta.y * Math.sin(p.heading),
            -pDelta.x * Math.sin(p.heading) + pDelta.y * Math.cos(p.heading),
            pDelta.heading
        );


        double deltaAng = lastOrientation - orientation - pDeltaRelative.heading;
        Pose2d expectedNewBlockPose = new Pose2d( // Calculated from solely poses
            (blockPos.x - pDeltaRelative.x) * Math.cos(deltaAng) + (blockPos.y - pDeltaRelative.y) * Math.sin(deltaAng),
            (blockPos.x - pDeltaRelative.x) * Math.sin(deltaAng) + (blockPos.y - pDeltaRelative.y) * Math.cos(deltaAng),
            blockPos.heading + lastOrientation - orientation + pDeltaRelative.heading
        );
        TelemetryUtil.packet.put("p.x", p.x);
        TelemetryUtil.packet.put("p.y", p.y);
        TelemetryUtil.packet.put("p.heading", p.heading);
        TelemetryUtil.packet.put("Expected X", expectedNewBlockPose.x);
        TelemetryUtil.packet.put("Expected Y", expectedNewBlockPose.y);
        TelemetryUtil.packet.put("Expected Heading", expectedNewBlockPose.heading);
        TelemetryUtil.packet.put("pDelta X", pDelta.x);
        TelemetryUtil.packet.put("pDelta Y", pDelta.y);
        TelemetryUtil.packet.put("pDelta Heading", pDelta.heading);
        TelemetryUtil.packet.put("LL angle", orientation);
        TelemetryUtil.packet.put("deltaAng", deltaAng);
        TelemetryUtil.packet.put("orientation", orientation);

        // Update based on ONLY change in robot position
        if (result == null ||
            result.getStaleness() > 100 ||
            result.getColorResults().isEmpty() ||
            // Clamp
            Math.abs(result.getColorResults().get(0).getTargetXDegrees()) > maxAngX ||
            Math.abs(result.getColorResults().get(0).getTargetYDegrees()) > maxAngY) {

            blockPos = expectedNewBlockPose.clone();

            sumDetections = 0;
        } else { // We have a valid result. Now we can update with both change according to dt and change according to limelight
            // Post processing. Get new block x, y, and heading
            ColorResult cr = result.getColorResults().get(0);

            double x = getInchesX(cr.getTargetYDegrees());
            double y = getInchesY(-cr.getTargetXDegrees());

            Vector2 o = Vector2.staticrotate(offset, orientation);
            x += o.x;
            y += o.y;

            double heading = expectedNewBlockPose.heading;// - pDelta.heading;
            // Attempt to update heading value with the new value
            List<List<Double>> corners = cr.getTargetCorners();
            if (corners.size() == 4) { // I don't care enough to get this to work with stupid detections
                // THIS FORMAT IS SO HORRID I'M SORRY BUT I'M TAKING THE EXTRA STEP TO CHANGE THE FORMAT - Eric
                Vector2[] vcorners = new Vector2[4];
                for (int i = 0; i < corners.size(); i++) {
                    vcorners[i] = new Vector2(corners.get(i).get(0), corners.get(i).get(1));
                }

                // Get the 2 longest
                int l0 = 0; // nyeh pointer !
                int l1 = 0;
                double largestDistance = Vector2.distance(vcorners[l0], vcorners[l1]);
                for (int i = 0; i < vcorners.length; i++) {
                    for (int j = i + 1; j < vcorners.length; j++) { // No gaf
                        double d = Vector2.distance(vcorners[i], vcorners[j]);
                        if (d >= largestDistance) {
                            l0 = i;
                            l1 = j;
                            largestDistance = d;
                        }
                    }
                }
                // Now find out which one is the closest for each
                int cl0 = -1;
                int cl1 = -1;
                double distcl0 = Double.MAX_VALUE;
                double distcl1 = Double.MAX_VALUE;
                for (int i = 0; i < vcorners.length; i++) {
                    double dl0 = Vector2.distance(vcorners[l0], vcorners[i]);
                    double dl1 = Vector2.distance(vcorners[l1], vcorners[i]);
                    if (i != l0 && (cl0 == -1 || dl0 < distcl0)) {
                        cl0 = i;
                        distcl0 = dl0;
                    }
                    if (i != l1 && (cl1 == -1 || dl1 < distcl1)) {
                        cl1 = i;
                        distcl1 = dl1;
                    }
                }

                // Average the 2 small sides
                double h1 = AngleUtil.mirroredClipAngleTolerence(Math.atan2(vcorners[l0].y - vcorners[cl0].y, vcorners[l0].x - vcorners[cl0].x), Math.toRadians(20));
                double h2 = AngleUtil.mirroredClipAngleTolerence(Math.atan2(vcorners[cl1].y - vcorners[l1].y, vcorners[cl1].x - vcorners[l1].x), Math.toRadians(20));
                heading = (h1 + h2) / 2 + lastOrientation - orientation;

                sumDetections++;
            }

            heading = AngleUtil.mirroredClipAngleTolerence(heading, Math.toRadians(20));

            // If robot is moving very fast then it will only use drivetrain translational values to calculate new block pos
            Vector2 velocityVector = new Vector2(
                robot.sensors.getVelocity().x,
                robot.sensors.getVelocity().y
            );
            double weightedAvg = velocityVector.mag() / Drivetrain.maxVelocity;

            blockPos.x = expectedNewBlockPose.x * weightedAvg + x * (1 - weightedAvg);
            blockPos.y = expectedNewBlockPose.y * weightedAvg + y * (1 - weightedAvg);
            blockPos.heading = expectedNewBlockPose.heading * weightedAvg + heading * (1 - weightedAvg);

            TelemetryUtil.packet.put("weightedAvg", weightedAvg);
            TelemetryUtil.packet.put("velocity", velocityVector.mag());

            // Get the pose error (expected without limelight vs with limelight)
            errorX = errorX * 0.2 + (blockPos.x - expectedNewBlockPose.x) * 0.8;
            errorY = errorY * 0.2 + (blockPos.y - expectedNewBlockPose.y) * 0.8;
            errorHeading = errorHeading * 0.2 + (blockPos.heading - expectedNewBlockPose.heading) * 0.8;
        }

        if (useExpected)
            blockPos = expectedNewBlockPose.clone();

        // This is fine because detecting turning on would update this value properly
        lastPosition = robot.sensors.getOdometryPosition().clone();

        lastOrientation = orientation;
    }

    /**
     * Starts detection and uses robot position deltas
     */
    public void startDetection() {
        lastPosition = robot.sensors.getOdometryPosition().clone();
        blockPos = new Pose2d(0, 0, 0);
        detecting = true;
        lowPassError = sumDetections = 0;
    }

    /**
     * Stops detection
     */
    public void stopDetection() {
        detecting = true;
    }

    /**
     * @return Robot centric value
     */
    public Pose2d getBlockPos() {
        return blockPos;
    }

    public void setBlock(Block block) {
        ll.pipelineSwitch(block.pipelineIndex);
    }

    public void setOffset(Vector2 v) {
        offset = v;
    }

    public void resetConcurrentDetections() {
        sumDetections = 0;
    }

    public int concurrentDetections() {
        return sumDetections;
    }

    private double getInchesX(double x) {
        return 0.0162 + 0.114 * x + 1.76e-4 * x * x;
    }

    private double getInchesY(double y) {
        return 0.203 + 0.123 * y + -6.55e-5 * y * y;
    }

    public double getLowPassError() {
        return lowPassError;
    }

    public double getErrorX() {
        return errorX;
    }

    public double getErrorY() {
        return errorY;
    }

    public double getErrorHeading() {
        return errorHeading;
    }

    public void setNewOrientation(double ang) {
        if (lastOrientation == Double.MAX_VALUE) {
            lastOrientation = ang;
        }
        orientation = ang;
    }
}
