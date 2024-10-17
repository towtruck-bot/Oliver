package org.firstinspires.ftc.teamcode.sensors;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;


public class Sensors {
    private LynxModule controlHub, expansionHub;
    private final HardwareQueue hardwareQueue;
    private final HardwareMap hardwareMap;
    private Robot robot;

    //private IMU imu;
    private int[] odometry = new int[] {0,0,0};

    private int slidesEncoder;
    private double slidesVelocity;

    private int intakeExtensionEncoder;

    private final AnalogInput[] analogEncoders = new AnalogInput[2];
    public double[] analogVoltages = new double[analogEncoders.length];

    private double voltage;

    private SparkFunOTOS otos;
    private double otosHeading = 0;
    private long numOtosLoops = 0;
    private double otosIntegral = 0;
    private double lastOtosIntegral = 0;
    private SparkFunOTOS.Pose2D sparkPose = new SparkFunOTOS.Pose2D();

    private double leftFrontMotorCurrent, leftRearMotorCurrent, rightRearMotorCurrent, rightFrontMotorCurrent;

    public static double voltageK = 0.3;

    public Sensors(Robot robot) {
        this.hardwareMap = robot.hardwareMap;
        this.hardwareQueue = robot.hardwareQueue;
        this.robot = robot;

        otos = hardwareMap.get(SparkFunOTOS.class, "sparkfunSensor");
        otos.setLinearUnit(SparkFunOTOS.LinearUnit.INCHES);
        otos.setAngularUnit(SparkFunOTOS.AngularUnit.RADIANS);
        otos.calibrateImu();
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D( -3.333,2.9375, 0);
        otos.setOffset(offset);
        otos.setLinearScalar(1.010);
        otos.setAngularScalar(0.992);
        otos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);



        initSensors(hardwareMap);
    }

    private void initSensors(HardwareMap hardwareMap) {
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void update() {
        updateControlHub();
        updateExpansionHub();
        updateTelemetry();
    }

    public void setOtosHeading(double heading) {
        otos.setPosition(new SparkFunOTOS.Pose2D( -3.333,2.9375, heading));
        lastOtosIntegral = otosHeading = otosIntegral = heading;
    }

    private double imuUpdateTime = 15;
    public double timeTillNextIMUUpdate = imuUpdateTime;
    public boolean imuJustUpdated = false;

    private double voltageUpdateTime = 5000;
    long lastVoltageUpdatedTime = System.currentTimeMillis();

    private double huskyUpdateTime = 100;
    long lastHuskyLensUpdatedTime = System.currentTimeMillis();
    public boolean huskyJustUpdated = false;

    private void updateControlHub() {
        odometry[0] = ((PriorityMotor) hardwareQueue.getDevice("leftFront")).motor[0].getCurrentPosition(); // left (0)
        odometry[1] = ((PriorityMotor) hardwareQueue.getDevice("rightRear")).motor[0].getCurrentPosition(); // right (3)
        odometry[2] = ((PriorityMotor) hardwareQueue.getDevice("leftRear")).motor[0].getCurrentPosition(); // back (1)

        long currTime = System.currentTimeMillis();

        double lastOtosHeading = otosHeading;
        otosHeading = otos.getHeading() * -1;
        lastOtosIntegral = otosIntegral;
        otosIntegral += AngleUnit.normalizeRadians(lastOtosHeading - otosHeading) * 0.998197;
        TelemetryUtil.packet.put("OTOSHeading", Math.toDegrees(otosHeading));
        TelemetryUtil.packet.put("OTOSIntegral", Math.toDegrees(otosIntegral));

        if (currTime - lastVoltageUpdatedTime > voltageUpdateTime) {
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = currTime;
        }

        slidesEncoder = ((PriorityMotor) hardwareQueue.getDevice("rightFront")).motor[0].getCurrentPosition() * -1;
        slidesVelocity = ((PriorityMotor) hardwareQueue.getDevice("rightFront")).motor[0].getVelocity() * -1;

        this.intakeExtensionEncoder = this.robot.intake.intakeExtensionMotor.motor[0].getCurrentPosition();
        TelemetryUtil.packet.put("Extendo position", this.getIntakeExtensionPosition());
    }

    private void updateExpansionHub() {
        try {
        }
        catch (Exception e) {
            Log.e("******* Error due to ", e.getClass().getName());
            e.printStackTrace();
            Log.e("******* fail", "expansion hub failed");
        }
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("voltage", voltage);
    }

    public int[] getOdometry() {
        return odometry;
    }

    public int getSlidesPos() {
        return slidesEncoder;
    }
    public double getSlidesVelocity() {
        return slidesVelocity;
    }

    /**
     * Gets the intake extension slides' position. -- Daniel
     * @return the intake extension slides' position, in inches
     */
    public double getIntakeExtensionPosition() {
        final double inchesPerTick = 0.04132142857142857; // TODO This value is TEMPORARY (Centerstage/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/deposit/Slides.java)
        return this.intakeExtensionEncoder * inchesPerTick;
    }

    public double getVoltage() { return voltage; }

    public double getLastOtosHeading() {
        return lastOtosIntegral;
    }

    public double getOtosHeading() {
        return otosIntegral;
    }

    public void updateDrivetrainMotorCurrents() {
        leftFrontMotorCurrent = robot.drivetrain.leftFront.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
        leftRearMotorCurrent = robot.drivetrain.leftRear.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
        rightRearMotorCurrent = robot.drivetrain.rightRear.motor[0].getCurrent(CurrentUnit.MILLIAMPS);
        rightFrontMotorCurrent = robot.drivetrain.rightFront.motor[0].getCurrent(CurrentUnit.MILLIAMPS);

        TelemetryUtil.packet.put("leftFrontMotorCurrent", leftFrontMotorCurrent);
        TelemetryUtil.packet.put("leftRearMotorCurrent", leftRearMotorCurrent);
        TelemetryUtil.packet.put("rightRearMotorCurrent", rightRearMotorCurrent);
        TelemetryUtil.packet.put("rightFrontMotorCurrent", rightFrontMotorCurrent);
    }

    private double previousAngle = 0.0;
    private int numRotations = 0;
    private void addToCumulativeHeading(double angle) {
        if (Math.abs(angle-previousAngle) >= Math.toRadians(180)) {
            numRotations += Math.signum(previousAngle);
        }
        previousAngle = angle;
    }
}

