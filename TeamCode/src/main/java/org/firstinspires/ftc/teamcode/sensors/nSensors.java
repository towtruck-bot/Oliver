package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class nSensors {
    private HardwareMap hardwareMap;
    private HardwareQueue hardwareQueue;
    private GoBildaPinpointDriver odometry;

    private double voltage;
    private double voltageUpdateTime = 5000;
    private long lastVoltageUpdatedTime = System.currentTimeMillis();

    private int slidesEncoder;
    public static double slidesInchesPerTick = 0.01773835920177383;

    private int extendoEncoder;
    public static double extendoInchesPerTick = 0.04293545803;


    public nSensors(Robot robot){
        this.hardwareMap = robot.hardwareMap;
        this.hardwareQueue = robot.hardwareQueue;

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odometry.setOffsets(-73.025, -48.26);

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void update(){
        odometry.update();

        slidesEncoder = ((PriorityMotor) hardwareQueue.getDevice("slidesMotor")).motor[0].getCurrentPosition();
        extendoEncoder = ((PriorityMotor) hardwareQueue.getDevice("intakeExtensionMotor")).motor[0].getCurrentPosition();

        if (System.currentTimeMillis() - lastVoltageUpdatedTime > voltageUpdateTime) {
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = System.currentTimeMillis() ;
        }

        updateTelemetry();
    }

    public void setOdometryPosition(double x, double y, double heading){
        odometry.setPosition(new Pose2d(x, y, heading));
    }

    public Pose2d getOdometryPosition(){
        return odometry.getPosition();
    }

    public void setHeading(double heading){
        odometry.setPosition(new Pose2d(odometry.getPosX(), odometry.getPosY(), heading));
    }

    public double getHeading(){
        return odometry.getHeading();
    }

    public double getSlidesPos(){
        return slidesEncoder * slidesInchesPerTick;
    }

    public double getExtendoPos(){
        return extendoEncoder * extendoInchesPerTick;
    }

    public double getVoltage(){
        return voltage;
    }

    private void updateTelemetry(){
        TelemetryUtil.packet.put("voltage", voltage);

        TelemetryUtil.packet.put("Extendo position", getExtendoPos());
        TelemetryUtil.packet.put("Extendo encoder", this.extendoEncoder);

        TelemetryUtil.packet.put("Slides position", getSlidesPos());
        TelemetryUtil.packet.put("Slides encoder", this.slidesEncoder);
    }
}
