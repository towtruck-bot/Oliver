package org.firstinspires.ftc.teamcode.sensors;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;


public class Sensors {
    private LynxModule controlHub, expansionHub;
    private final HardwareQueue hardwareQueue;
    private final HardwareMap hardwareMap;
    private Robot robot;

    public enum BlockColor {
        NONE,
        RED,
        BLUE,
        YELLOW,
    }

    //private IMU imu;
    private int[] odometry = new int[] {0,0,0};
    private GoBildaPinpointDriver odo;

    private int slidesEncoder;
    private double slidesVelocity;
    public static final double slidesInchesPerTick = 0.01773835920177383;

    private double extendoEncoder;

    public final DigitalChannel intakeColorSensorR;
    public final DigitalChannel intakeColorSensorB;
    private BlockColor intakeColor = BlockColor.NONE;

    private final AnalogInput[] analogEncoders = new AnalogInput[2];
    public double[] analogVoltages = new double[analogEncoders.length];

    private double voltage;

    //private SparkFunOTOS otos;
    private double otosHeading = 0;
    private long numOtosLoops = 0;
    private double otosIntegral = 0;
    private double lastOtosIntegral = 0;
    //private SparkFunOTOS.Pose2D sparkPose = new SparkFunOTOS.Pose2D();

    private double leftFrontMotorCurrent, leftRearMotorCurrent, rightRearMotorCurrent, rightFrontMotorCurrent;

    public static double voltageK = 0.3;

    public static double blueAlpha  = 0.125, redAlpha = 0.125, yellowAlpha = 0.125;
    public double blueConfidence, yellowConfidence, redConfidence;
    public static double blueThreshold = 0.8, redThreshold = 0.8, yellowThreshold = 0.8;

    public Sensors(Robot robot) {
        this.hardwareMap = robot.hardwareMap;
        this.hardwareQueue = robot.hardwareQueue;
        this.robot = robot;

        this.odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        //TODO: Tune offsets
        odo.setOffsets(-84.0, -63); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setPosition(new Pose2d(0.0, 0.0, 0.0));
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        this.intakeColorSensorR = this.robot.hardwareMap.get(DigitalChannel.class, "intakeColorSensorR");
        this.intakeColorSensorR.setMode(DigitalChannel.Mode.INPUT);
        this.intakeColorSensorB = this.robot.hardwareMap.get(DigitalChannel.class, "intakeColorSensorB");
        this.intakeColorSensorB.setMode(DigitalChannel.Mode.INPUT);

        this.blueConfidence = 0.0;
        this.yellowConfidence = 0.0;
        this.redConfidence = 0.0;

        initSensors(hardwareMap);
    }

    private void initSensors(HardwareMap hardwareMap) {
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        expansionHub = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        expansionHub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void update() {
        updateControlHub();
        updateExpansionHub();
        updateTelemetry();
    }

    public void setOtosHeading(double heading) {
        //otos.setPosition(new SparkFunOTOS.Pose2D( -3.333,2.9375, heading));
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

        odo.update();

        long currTime = System.currentTimeMillis();

        double lastOtosHeading = otosHeading;
        //otosHeading = otos.getHeading() * -1;
        lastOtosIntegral = otosIntegral;
        otosIntegral += AngleUnit.normalizeRadians(lastOtosHeading - otosHeading) * 0.998197;

        if (currTime - lastVoltageUpdatedTime > voltageUpdateTime) {
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = currTime;
        }
    }

    private void updateExpansionHub() {
        try {
            slidesEncoder = ((PriorityMotor) hardwareQueue.getDevice("slidesMotor")).motor[0].getCurrentPosition();
            slidesVelocity = ((PriorityMotor) hardwareQueue.getDevice("slidesMotor")).motor[0].getVelocity();
            extendoEncoder = ((PriorityMotor) hardwareQueue.getDevice("intakeExtensionMotor")).motor[0].getCurrentPosition();

            boolean colorSensor1 = intakeColorSensorR.getState();
            boolean colorSensor0 = intakeColorSensorB.getState();
            calculateConfidence(colorSensor1, colorSensor0);
        }
        catch (Exception e) {
            Log.e("******* Error due to ", e.getClass().getName());
            e.printStackTrace();
            Log.e("******* fail", "expansion hub failed");
        }
    }

    public void updateTelemetry() {
        TelemetryUtil.packet.put("voltage", voltage);

        TelemetryUtil.packet.put("Extendo position", this.getExtendoPosition());
        TelemetryUtil.packet.put("Extendo encoder", this.extendoEncoder);

        TelemetryUtil.packet.put("Slides position", this.getSlidesPosition());
        TelemetryUtil.packet.put("Slides encoder", this.slidesEncoder);

        TelemetryUtil.packet.put("Intake color", this.intakeColor.toString());
    }

    public int[] getOdometry() {
        return odometry;
    }

    public Pose2d getPosition(){return odo.getPosition();}

    public double getSlidesVelocity() {
        return slidesVelocity * slidesInchesPerTick;
    }

    /**
     * Gets the vertical slides' position. -- Daniel
     * @return the vertical slides' position, in inches
     */
    public double getSlidesPosition() {
        return this.slidesEncoder * slidesInchesPerTick;
    }

    /**
     * Gets the intake extension slides' position. -- Daniel
     * @return the intake extension slides' position, in inches
     */

    final double inchesPerTick = 0.0409*1.0497667; //0.0409
    public double getExtendoPosition() {
        return this.extendoEncoder * inchesPerTick;
    }

    /**
     * Gets the color detected by the intake. -- Daniel
     * @return the color detected by the intake (NONE, RED, BLUE, YELLOW)
     */
    public BlockColor getIntakeColor() {
        return this.intakeColor;
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
    }

    private double previousAngle = 0.0;
    private int numRotations = 0;
    private void addToCumulativeHeading(double angle) {
        if (Math.abs(angle-previousAngle) >= Math.toRadians(180)) {
            numRotations += Math.signum(previousAngle);
        }
        previousAngle = angle;
    }

    public void calculateConfidence(boolean colorSensor1, boolean colorSensor0){
        //1 -> R, 0 -> B
        //yellow = 3, red = 2, blue = 1, none = 0
        int colorState = (colorSensor1?2:0) + (colorSensor0?1:0);
        blueConfidence *= (1 - blueAlpha);
        redConfidence *= (1 - redAlpha);
        yellowConfidence *= (1 - yellowAlpha);

        switch (colorState) {
            case 0: break;
            case 1:
                blueConfidence += blueAlpha;
                break;
            case 2:
                redConfidence += redAlpha;
                //Due to sometimes yellow flags as red
                yellowConfidence += 0.5 * yellowAlpha;
                break;
            case 3:
                yellowConfidence += yellowAlpha;
                break;
        }

        if(blueConfidence > blueThreshold){
            intakeColor = BlockColor.BLUE;
        }else if(redConfidence > redThreshold){
            intakeColor = BlockColor.RED;
        }else if(yellowConfidence > yellowThreshold){
            intakeColor = BlockColor.YELLOW;
        }else{
            intakeColor = BlockColor.NONE;
        }

        TelemetryUtil.packet.put("Blue Confidence", blueConfidence);
        TelemetryUtil.packet.put("Red Confidence", redConfidence);
        TelemetryUtil.packet.put("Yellow Confidence", yellowConfidence);
        TelemetryUtil.packet.put("Detected Color", intakeColor.toString());
    }
}

