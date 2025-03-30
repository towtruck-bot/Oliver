package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;

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

    private Limelight3A ll;
    private Block block;
    private Pose2d blockPos; // Robot centric vaue
    private Pose2d lastPosition;
    private Robot robot;
    private boolean detecting = false;
    private Vector2 offset = new Vector2(0, 0);
    public static int pollRate = 100;
    public boolean blockDetected = false;
    private int sumDetections = 0;

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

        Vector2 pNewBlockPose = new Vector2( // Calculated from solely poses
            (blockPos.x - pDelta.x) * Math.cos(pDelta.heading) - (blockPos.y - pDelta.y) * Math.sin(pDelta.heading),
            (blockPos.x - pDelta.x) * Math.sin(pDelta.heading) + (blockPos.y - pDelta.y) * Math.cos(pDelta.heading)
        );

        // Update based on ONLY change in robot position
        if (result == null ||
            result.getStaleness() > 100 ||
            result.getColorResults().isEmpty() ||
                // Clamp
            Math.abs(result.getColorResults().get(0).getTargetXDegrees()) > 20 ||
            Math.abs(result.getColorResults().get(0).getTargetYDegrees()) > 20) {

            blockPos.x = pNewBlockPose.x;
            blockPos.y = pNewBlockPose.y;
            blockPos.heading = blockPos.heading - pDelta.heading; // Is this right??

            blockDetected = false;
            sumDetections = 0;
        } else { // We have a valid result. Now we can update with both change according to dt and change according to limelight
            // We can use IMU data to do some wackyyy stuff dudeee
            YawPitchRollAngles angles = result.getBotpose().getOrientation();

            // Post processing. Get new block x, y, and heading
            ColorResult cr = result.getColorResults().get(0);

            double x = getInchesX(cr.getTargetYDegrees());
            double y = getInchesY(-cr.getTargetXDegrees());
            double heading = blockPos.heading;// - pDelta.heading;
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
                double largestDistance = 0;
                for (int i = 1; i < vcorners.length; i++) {
                    for (int j = 0; j < vcorners.length; j++) { // No gaf
                        double d = Vector2.distance(vcorners[i], vcorners[j]);
                        if (d > largestDistance) {
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
                    if (i != l0 && (cl0 == -1 || dl0 < distcl0))
                        cl0 = i;
                    if (i != l1 && (cl1 == -1 || dl1 < distcl1))
                        cl1 = i;
                }

                // Average the 2 small sides
                double h1 = Math.atan2(vcorners[l0].y - vcorners[cl0].y, vcorners[l0].x - vcorners[cl0].x);
                double h2 = Math.atan2(vcorners[l1].y - vcorners[cl1].y, vcorners[l1].x - vcorners[cl1].x) + Math.toRadians(270);
                TelemetryUtil.packet.put("h1 value", Math.toDegrees(h1));
                TelemetryUtil.packet.put("h2 value", Math.toDegrees(h2));
                heading = (h1 + h2) / 2;
                /*-
                        p.heading + angles.getYaw(AngleUnit.RADIANS);*/

                blockDetected = true;
                sumDetections++;
            }

            while (heading > Math.PI / 2) {
                heading -= Math.PI;
            }
            while (heading < -Math.PI / 2) {
                heading += Math.PI;
            }

            // If robot is moving very fast then it will only use drivetrain translational values to calculate new block pos
            double weightedAvg = robot.sensors.getVelocity().toVec3().getMag() / Drivetrain.maxVelocity;
            blockPos.x = pNewBlockPose.x * weightedAvg + x * (1 - weightedAvg);
            blockPos.y = pNewBlockPose.y * weightedAvg + y * (1 - weightedAvg);
            // Low pass filter
            blockPos.heading = blockPos.heading * 0.8 + heading * 0.2;//(blockPos.heading - pDelta.heading) * weightedAvg + heading * (1 - weightedAvg);
        }

        // This is fine because detecting turning on would update this value properly
        lastPosition = robot.sensors.getOdometryPosition().clone();

        Vector2 o = Vector2.staticrotate(offset, pDelta.heading);
        blockPos.x += o.x;
        blockPos.y += o.y;
    }

    /**
     * Starts detection and uses robot position deltas
     */
    public void startDetection() {
        lastPosition = robot.sensors.getOdometryPosition().clone();
        blockPos = new Pose2d(0, 0, 0);
        detecting = true;
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
}
