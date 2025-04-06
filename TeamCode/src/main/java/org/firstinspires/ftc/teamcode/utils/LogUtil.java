package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;


@Config
public class LogUtil {
    private static Datalogger datalogger = null;

    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    //public static Datalogger.GenericField loopTime = new Datalogger.GenericField("loopTime");
    public static Datalogger.GenericField robotState = new Datalogger.GenericField("robotState");
    public static Datalogger.GenericField intakeState = new Datalogger.GenericField("intakeState");
    public static Datalogger.GenericField depositState = new Datalogger.GenericField("depositState");
    public static Datalogger.GenericField extendoCurrentPos = new Datalogger.GenericField("extendoCurrentPos");
    public static Datalogger.GenericField extendoTargetPos = new Datalogger.GenericField("extendoTargetPos");
    public static Datalogger.GenericField intakeClawRotationAngle = new Datalogger.GenericField("intakeClawRotationAngle");
    public static Datalogger.GenericField intakeTurretRotationAngle = new Datalogger.GenericField("intakeTurretRotationAngle");
    public static Datalogger.GenericField intakeClawGrab = new Datalogger.GenericField("intakeClawGrab");
    public static Datalogger.GenericField slidesCurrentPos = new Datalogger.GenericField("slidesCurrentPos");
    public static Datalogger.GenericField slidesTargetPos = new Datalogger.GenericField("slidesTargetPos");
    public static Datalogger.GenericField driveState = new Datalogger.GenericField("driveState");
    public static Datalogger.GenericField driveCurrentX = new Datalogger.GenericField("driveCurrentX");
    public static Datalogger.GenericField driveCurrentY = new Datalogger.GenericField("driveCurrentY");
    public static Datalogger.GenericField driveCurrentAngle = new Datalogger.GenericField("driveCurrentAngle");
    public static Datalogger.GenericField driveTargetX = new Datalogger.GenericField("driveTargetX");
    public static Datalogger.GenericField driveTargetY = new Datalogger.GenericField("driveTargetY");
    public static Datalogger.GenericField driveTargetAngle = new Datalogger.GenericField("driveTargetAngle");

    private static int loopCountBeforeWrite;

    public static boolean DISABLED = false;
    public static boolean stateTransition = false;

    public static void reset() {
        if (datalogger != null) {
            datalogger = null;
        }
    }

    public static void init() {
        loopCountBeforeWrite = 0;

        long timeNow = System.currentTimeMillis();
        String fileName = "Log_" + timeNow + "_"
            + new SimpleDateFormat("yyyy-MM-dd_HH.mm.ss", Locale.US).format(new Date(timeNow))
            + "_" + Globals.RUNMODE.toString();
        TelemetryUtil.packet.put("Log filename", fileName);

        if (datalogger != null) throw new IllegalStateException("LogUtil was already initialized");
        if (DISABLED) return;

        datalogger = new Datalogger.Builder()
            // Pass through the filename
            .setFilename(fileName)
            // Request an automatic timestamp field
            .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)
            // Tell it about the fields we care to log.
            // Note that order *IS* important here! The order in which we list
            // the fields is the order in which they will appear in the log.
            .setFields(
                intakeState,
                depositState,
                extendoCurrentPos,
                extendoTargetPos,
                intakeClawRotationAngle,
                intakeTurretRotationAngle,
                intakeClawGrab,
                slidesCurrentPos,
                slidesTargetPos,
                driveCurrentX,
                driveCurrentY,
                driveCurrentAngle,
                driveState,
                driveTargetX,
                driveTargetY,
                driveTargetAngle
            )
            .build();
    }

    public static void send() {
        if (datalogger == null) return;
        --loopCountBeforeWrite;
        if (loopCountBeforeWrite <= 0 || stateTransition) {
            datalogger.writeLine();
            loopCountBeforeWrite = 25;
            stateTransition = false;
        }
    }
}
