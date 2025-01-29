package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;

public class Sensors {
    private HardwareMap hardwareMap;
    private HardwareQueue hardwareQueue;
    private GoBildaPinpointDriver odometry;

    private double voltage;
    private double voltageUpdateTime = 5000;
    private long lastVoltageUpdatedTime = System.currentTimeMillis();

    private int slidesEncoder;
    private double slidesVel;
    public static double slidesInchesPerTick = 0.01773835920177383;

    private int extendoEncoder;
    public static double extendoInchesPerTick = 0.04293545803;

    private final AnalogInput[] analogEncoders = new AnalogInput[2];
    public double[] analogVoltages = new double[analogEncoders.length];

    public Sensors(Robot robot){
        this.hardwareMap = robot.hardwareMap;
        this.hardwareQueue = robot.hardwareQueue;

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // Set tracking point offsets here. in mm
        // new x? 7.6, new y? 6.75?
        // old x? 7.75, old y? 6.5
        odometry.setOffsets(7.6, 6.75);
        odometry.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
    }

    public void update(){
        odometry.update();

        slidesEncoder = ((PriorityMotor) hardwareQueue.getDevice("slidesMotor")).motor[0].getCurrentPosition();
        slidesVel = ((PriorityMotor) hardwareQueue.getDevice("slidesMotor")).motor[0].getVelocity();
        extendoEncoder = ((PriorityMotor) hardwareQueue.getDevice("intakeExtensionMotor")).motor[0].getCurrentPosition();

        if (System.currentTimeMillis() - lastVoltageUpdatedTime > voltageUpdateTime) {
            voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            lastVoltageUpdatedTime = System.currentTimeMillis() ;
        }

        updateTelemetry();
    }

    public void resetPosAndIMU(){
        odometry.resetPosAndIMU();
    }

    public void setOffsets(double x, double y){
        odometry.setOffsets(x, y);
    }

    public void setOdometryPosition(double x, double y, double heading){
        odometry.setPosition(new Pose2d(x, y, heading));
    }

    public void setOdometryPosition(Pose2d pose2d){
        odometry.setPosition(pose2d);
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

    public Pose2d getVelocity() {return odometry.getVelocity();}

    public double getSlidesPos(){
        return slidesEncoder * slidesInchesPerTick;
    }

    public double getSlidesVel(){
        return slidesEncoder * slidesVel;
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
