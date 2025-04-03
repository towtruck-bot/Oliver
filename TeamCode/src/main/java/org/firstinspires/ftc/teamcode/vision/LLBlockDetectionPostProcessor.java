package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;

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
    private final Limelight3A ll;
    private Block block;
    private Pose2d blockPos; // Robot centric vaue
    private Pose2d lastRobotPosition;
    private final Robot robot;
    private boolean detecting = false;
    private Vector2 offset = new Vector2(0, 0);
    public static int pollRate = 100;
    public static int maxAngX = 14;
    public static int maxAngY = 14;
    private double lastOrientation = 0;
    private double orientation = 0;
    private Pose2d lastBlockPosition;
    private Vector2 deltaOffset;
    private double velocityLowPass = 0;
    private double lastLoop = System.currentTimeMillis();
    private boolean firstOrientation = false;
    private int detections = 0;

    public LLBlockDetectionPostProcessor(Robot robot) {
        ll = robot.hardwareMap.get(Limelight3A.class, "limelight");
        block = Block.YELLOW;
        ll.setPollRateHz(pollRate);
        blockPos = new Pose2d(0, 0, 0);
        lastBlockPosition = null;
        lastRobotPosition = null;
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
        Canvas c = TelemetryUtil.packet.fieldOverlay();
        c.setStroke("#444400");
        c.strokeCircle(blockPos.x, blockPos.y, 3);
        c.strokeLine(blockPos.x, blockPos.y, blockPos.x + 5 * Math.sin(blockPos.heading), blockPos.y + 5 * Math.cos(blockPos.heading));

        if (!detecting)
            return;

        int newDetections = detections;

        if (!ll.isConnected()) {
            Log.e("ERROR BIG", "Limelight Broke");
        }

        long loopStart = System.currentTimeMillis();

        LLResult result = ll.getLatestResult();

        // Get robot based pose delta
        Pose2d p = robot.sensors.getOdometryPosition();
        Pose2d pDelta = new Pose2d( // Pose delta
            p.x - lastRobotPosition.x,
            p.y - lastRobotPosition.y,
            p.heading - lastRobotPosition.heading
        );
        Pose2d pDeltaRelative = new Pose2d(
            pDelta.x * Math.cos(p.heading) + pDelta.y * Math.sin(p.heading),
            -pDelta.x * Math.sin(p.heading) + pDelta.y * Math.cos(p.heading),
            pDelta.heading
        );


        double deltaAng = lastOrientation - orientation - pDeltaRelative.heading;
        Pose2d expectedNewBlockPose = new Pose2d( // Calculated from solely poses
            (blockPos.x - pDeltaRelative.x - deltaOffset.x) * Math.cos(deltaAng) + (blockPos.y - pDeltaRelative.y - deltaOffset.y) * Math.sin(deltaAng),
            (blockPos.x - pDeltaRelative.x - deltaOffset.x) * Math.sin(deltaAng) + (blockPos.y - pDeltaRelative.y - deltaOffset.y) * Math.cos(deltaAng),
            blockPos.heading + lastOrientation - orientation + pDeltaRelative.heading
        );

        // Update based on ONLY change in robot position
        if (result == null ||
            result.getStaleness() > 100 ||
            result.getColorResults().isEmpty() ||
            // Clamp
            Math.abs(result.getColorResults().get(0).getTargetXDegrees()) > maxAngX ||
            Math.abs(result.getColorResults().get(0).getTargetYDegrees()) > maxAngY) {

            if (detections >= 1)
                blockPos = expectedNewBlockPose.clone();
        } else { // We have a valid result. Now we can update with both change according to dt and change according to limelight
            // Post processing. Get new block x, y, and heading
            ColorResult cr = result.getColorResults().get(0);

            double x = getInchesX(cr.getTargetYDegrees());
            double y = getInchesY(-cr.getTargetXDegrees());

            x += offset.x;
            y += offset.y;

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
                heading = (h1 + h2) / 2 - orientation;
                newDetections++;
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

        }

        if (lastBlockPosition == null)
            lastBlockPosition = blockPos.clone();

        // Get velocity value and low pass filter it
        Vector2 blockVelocity = new Vector2( // Technically it doesnt move
            (blockPos.x - lastBlockPosition.x) / (loopStart - lastLoop) / 1.0e-3,
            (blockPos.y - lastBlockPosition.y) / (loopStart - lastLoop) / 1.0e-3
        );
        TelemetryUtil.packet.put("loopDelta", (loopStart - lastLoop) * 1.0e-3);
        TelemetryUtil.packet.put("blockVelocity.x", blockVelocity.x);
        TelemetryUtil.packet.put("blockVelocity.y", blockVelocity.y);
        TelemetryUtil.packet.put("blockVelocity mag", blockVelocity.mag());
        TelemetryUtil.packet.put("velocityLowPass", velocityLowPass);
        velocityLowPass = blockVelocity.mag() * 0.6 + velocityLowPass * 0.4;
        if (detections <= 0)
            velocityLowPass = 0;
        detections = newDetections;

        // This is fine because detecting turning on would update this value properly
        lastRobotPosition = robot.sensors.getOdometryPosition().clone();

        lastOrientation = orientation;
        lastBlockPosition = blockPos.clone();
        lastLoop = loopStart;
    }

    /**
     * Starts detection and uses robot position deltas
     */
    public void startDetection() {
        lastRobotPosition = robot.sensors.getOdometryPosition().clone();
        blockPos = new Pose2d(0, 0, 0);
        lastBlockPosition = null;
        velocityLowPass = 0;
        detections = 0;
        lastOrientation = 0;
        orientation = 0;
        detecting = true;
        firstOrientation = true;
        lastLoop = System.currentTimeMillis();
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
        deltaOffset = new Vector2(
            offset.x - v.x,
            offset.y - v.y
        );
        offset = v;
    }

    private double getInchesX(double x) {
        return 0.0162 + 0.114 * x + 1.76e-4 * x * x;
    }

    private double getInchesY(double y) {
        return 0.203 + 0.123 * y + -6.55e-5 * y * y;
    }

    public void setNewOrientation(double ang) {
        if (firstOrientation) {
            lastOrientation = ang;
            firstOrientation = false;
        }
        orientation = ang;
    }

    public boolean isStable() {
        return velocityLowPass < 3 && detections >= 2; // Detections >= 2 allows us to have a velocity
    }

    public double getVelocityLowPass() {
        return velocityLowPass;
    }

    public boolean gottenFirstContact() {
        return detections >= 1;
    }
}
