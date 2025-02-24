package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.sensors.Sensors;
import org.firstinspires.ftc.teamcode.utils.DashboardUtil;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.PID;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.priority.PriorityMotor;
import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.Arrays;
import java.util.List;

public class nDrivetrain{
    public enum DriveState{
        GO_TO_POINT,
        ADJUST,
        BRAKE,
        WAIT_AT_POINT,
        DRIVE,
        IDLE
    }
    public DriveState state = DriveState.IDLE;

    private Robot robot;
    private Vision vision;
    private Sensors sensors;

    private PriorityMotor leftFront, leftRear, rightRear, rightFront;
    private final List<PriorityMotor> motors;

    public nDrivetrain(Robot robot, Vision vision){
        this.robot = robot;
        this.vision = vision;
        this.sensors = robot.sensors;

        leftFront = new PriorityMotor(
                robot.hardwareMap.get(DcMotorEx.class, "leftFront"),
                "leftFront",
                4, 5, 1.0, sensors
        );

        leftRear = new PriorityMotor(
                robot.hardwareMap.get(DcMotorEx.class, "leftRear"),
                "leftRear",
                4, 5, -1.0, sensors
        );
        rightRear = new PriorityMotor(
                robot.hardwareMap.get(DcMotorEx.class, "rightRear"),
                "rightRear",
                4, 5, sensors
        );
        rightFront = new PriorityMotor(
                robot.hardwareMap.get(DcMotorEx.class, "rightFront"),
                "rightFront",
                4, 5, sensors
        );

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        configureMotors();
        setMinPowersToOvercomeFriction();
    }

    private void configureMotors() {
        for (PriorityMotor motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.motor[0].getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.motor[0].setMotorType(motorConfigurationType);

            robot.hardwareQueue.addDevice(motor);
        }

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.motor[0].setDirection(DcMotor.Direction.REVERSE);
        leftRear.motor[0].setDirection(DcMotor.Direction.REVERSE);
    }

    private void setMode(DcMotor.RunMode runMode) {
        for (PriorityMotor motor : motors) {
            motor.motor[0].setMode(runMode);
        }
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (PriorityMotor motor : motors) {
            motor.motor[0].setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    // leftFront, leftRear, rightRear, rightFront
    double[] minPowersToOvercomeFriction = new double[] {
            0.3121803239920063,
            0.3533249418072871,
            0.36038420175052865,
            0.39695077434023707
    };

    private void setMinPowersToOvercomeFriction() {
        leftFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[0]);
        leftRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[1]);
        rightRear.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[2]);
        rightFront.setMinimumPowerToOvercomeStaticFriction(minPowersToOvercomeFriction[3]);

        for (PriorityMotor m : motors) {
            m.setMinimumPowerToOvercomeKineticFriction(0.195);
        }
    }

    private Pose2d targetPoint = new Pose2d(0.0, 0.0, 0.0);
    private double xError = 0.0, yError = 0.0, turnError = 0.0;

    public void update() {
        Pose2d estimate = sensors.getOdometryPosition();
        Globals.ROBOT_POSITION = new Pose2d(estimate.getX(), estimate.getY(), estimate.getHeading());
        Globals.ROBOT_VELOCITY = sensors.getVelocity();

        calculateErrors();
        updateTelemetry();

        switch(state) {
            case GO_TO_POINT:
                setMinPowersToOvercomeFriction();
                goToPoint();

                if (atPoint()) {
                    if(moveNear){
                        state = DriveState.ADJUST;
                    }

                    if (stop) {
                        state = DriveState.BRAKE;
                    }
                }
                break;
            case ADJUST:
                setMinPowersToOvercomeFriction();

                // Idea of this state is to allow the intake to extend while adjusting for any heading errors
                // Need to set new target values based on calculations of current position to determine new heading (write calculation method)

                goToPoint();


        }
    }

    private void calculateErrors() {
        double deltaX = (targetPoint.x - sensors.getOdometryPosition().x);
        double deltaY = (targetPoint.y - sensors.getOdometryPosition().y);

        xError = Math.cos(sensors.getOdometryPosition().heading) * deltaX + Math.sin(sensors.getOdometryPosition().heading) * deltaY;
        yError = -Math.sin(sensors.getOdometryPosition().heading) * deltaX + Math.cos(sensors.getOdometryPosition().heading) * deltaY;
        turnError = targetPoint.heading-sensors.getOdometryPosition().heading;

        while (Math.abs(turnError) > Math.PI ) {
            turnError -= Math.PI * 2 * Math.signum(turnError);
        }
    }

    private double maxPower;
    private boolean stop = false, slowDown = false, moveNear = false, adjust = false;

    public static PID xPID = new PID(0.147,0.0,0.026);
    public static PID yPID = new PID(0.15,0.0,0.025);
    public static PID turnPID = new PID(0.25,0.0,0.01);

    public static double xThreshold = 2.0;
    public static double yThreshold = 2.0;
    public static double turnThreshold = 4;

    public static double xEAThreshold = 16.0;
    public static double yEAThreshold = 16.0;
    public static double turnEAThreshold = 3.0;

    private double fwd, strafe, turn;

    // TODO: Implement a predictive PID
    private void goToPoint() {
        if (moveNear && atPoint()) {
            fwd = 0.0;
            strafe = 0.0;
        } else {
            fwd = xPID.update(xError, -maxPower, maxPower);
            strafe = yPID.update(yError, -maxPower, maxPower);
        }

        turn = turnPID.update(turnError, -maxPower, maxPower);

        Vector2 move = new Vector2(fwd, strafe);
        setMoveVector(move, turn);
    }

    private boolean atPoint() {
        if (moveNear) {
            return Math.abs(xError) < xEAThreshold && Math.abs(yError) < yEAThreshold && Math.abs(turnError) < turnEAThreshold;
        }

        return Math.abs(xError) < xThreshold && Math.abs(yError) < yThreshold && Math.abs(turnError) < turnThreshold;
    }

    private void setMoveVector(Vector2 moveVector, double turn) {
        double[] powers = {
                moveVector.x - turn - moveVector.y,
                moveVector.x - turn + moveVector.y,
                moveVector.x + turn - moveVector.y,
                moveVector.x + turn + moveVector.y
        };
        normalizeArray(powers);

        if (slowDown) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] = powers[i]*0.3;
            }
        }

        setMotorPowers(powers[0], powers[1], powers[2], powers[3]);
    }

    private void normalizeArray(double[] arr) {
        double largest = 1;
        for (int i = 0; i < arr.length; i++) {
            largest = Math.max(largest, Math.abs(arr[i]));
        }
        for (int i = 0; i < arr.length; i++) {
            arr[i] /= largest;
        }
    }

    private void setMotorPowers(double lf, double lr, double rr, double rf) {
        leftFront.setTargetPowerSmooth(lf);
        leftRear.setTargetPowerSmooth(lr);
        rightRear.setTargetPowerSmooth(rr);
        rightFront.setTargetPowerSmooth(rf);
    }

    private void updateTelemetry () {
        TelemetryUtil.packet.put("Drivetrain State", state);

        TelemetryUtil.packet.put("Drivetrain:: xError", xError);
        TelemetryUtil.packet.put("Drivetrain:: yError", yError);
        TelemetryUtil.packet.put("Drivetrain:: turnError (deg)", Math.toDegrees(turnError));
        TelemetryUtil.packet.put("Drivetrain:: xTarget", targetPoint.x);
        TelemetryUtil.packet.put("Drivetrain:: yTarget", targetPoint.y);

        Canvas canvas = TelemetryUtil.packet.fieldOverlay();

        DashboardUtil.drawRobot(canvas, targetPoint, "#ff00ff");
        canvas.setStroke("red");
        canvas.strokeCircle(targetPoint.x, targetPoint.y, xThreshold);
    }
}