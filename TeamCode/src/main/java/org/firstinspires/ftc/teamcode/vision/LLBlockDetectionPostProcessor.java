package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.AngleUtil;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
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
                // We're working with 4 I'm going to convert this to a data type that isn't bad because I have the time
                Vector2 longest0 = new Vector2(corners.get(0).get(0), corners.get(0).get(1));
                Vector2 longest1 = new Vector2(corners.get(1).get(0), corners.get(1).get(1));
                double dist = Vector2.distance(longest0, longest1);

                // Whatever its O(n^2) but its like 4 so who cares
                for (List<Double> l0 : corners) {
                    Vector2 p0 = new Vector2(l0.get(0), l0.get(1));
                    for (List<Double> l1 : corners) {
                        Vector2 p1 = new Vector2(l1.get(0), l1.get(1));
                        double d = Vector2.distance(p0, p1);
                        if (d > dist) {
                            longest0 = p0;
                            longest1 = p1;
                            dist = d;
                        }
                    }
                }

                // Theoredical angle of the longest dist the block should be 23.1985905 degrees
                heading = Math.atan2(longest0.y - longest1.y, longest0.x - longest1.x);
                heading -= Math.toRadians(23.1985905) * Math.signum(heading);
                /*-
                        p.heading + angles.getYaw(AngleUnit.RADIANS);*/

                blockDetected = true;
                sumDetections++;
            }

            // If robot is moving very fast then it will only use drivetrain translational values to calculate new block pos
            double weightedAvg = robot.sensors.getVelocity().toVec3().getMag() / Drivetrain.maxVelocity;
            blockPos.x = pNewBlockPose.x * weightedAvg + x * (1 - weightedAvg);
            blockPos.y = pNewBlockPose.y * weightedAvg + y * (1 - weightedAvg);
            blockPos.heading = AngleUtil.clipAngle(heading);//(blockPos.heading - pDelta.heading) * weightedAvg + heading * (1 - weightedAvg);

            while (blockPos.heading > Math.PI / 2) {
                blockPos.heading -= Math.PI;
            }
            while (blockPos.heading < -Math.PI / 2) {
                blockPos.heading += Math.PI;
            }

            if (Math.abs(blockPos.heading - Math.PI / 2) < Math.toRadians(35))
                blockPos.heading = 0;
            else if (Math.abs(blockPos.heading) < Math.toRadians(35))
                blockPos.heading = Math.PI / 2;
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
