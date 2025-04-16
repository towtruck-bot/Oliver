package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.ButtonToggle;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtil;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.ArrayList;

public class SubmersibleSelectionGUI {
    // Submersible 45in by 28in
    private double length = 27.5, width = 44.5;
    private int asciiLen = 19, asciiWidth = 15;
    private boolean[][] sub = new boolean[asciiLen][asciiWidth];

    private int cursorX = asciiLen / 2, cursorY = asciiWidth / 2;

    public ArrayList<Pose2d> getDriverSelect() {
        ArrayList<Pose2d> coords = new ArrayList<Pose2d>();

        for (int i = 0; i < asciiLen; i++) {
            for (int j = 0; j < asciiWidth; j++) {
                if (sub[i][j]) {
                    coords.add(
                            new Pose2d(
                                    (i + 0.5 - (double) asciiLen / 2.0) * length / (double) asciiLen,
                                    (j + 0.5 - (double) asciiWidth / 2.0) * width / (double) asciiWidth
                            )
                    );
                }
            }
        }

        return coords;
    }

    private boolean flash = true;
    private long timer = System.currentTimeMillis();
    private ButtonToggle down = new ButtonToggle(), up = new ButtonToggle(), left = new ButtonToggle(), right = new ButtonToggle(), a = new ButtonToggle();

    public void drawSub(Gamepad gamepad, Telemetry tele) {
        for (int j = 0; j < asciiWidth; j++) {
            String curr = "";
            for (int i = asciiLen - 1; i >= 0; i--) {
                if (flash && i == cursorX && j == cursorY) {
                    curr += "x";
                } else {
                    curr += sub[i][j] ? "#" : "_";
                }
            }

            tele.addData((j >= 10 ? "" : "0") + j, curr);
        }

        Canvas canvas = TelemetryUtil.packet.fieldOverlay();
        canvas.setFill("#ff80ff");
        for (Pose2d blockPos : getDriverSelect()) {
            canvas.fillCircle(blockPos.x, blockPos.y, 1);
        }

        tele.update();
        TelemetryUtil.sendTelemetry();

        if (down.isClicked(gamepad.dpad_down)) {
            cursorY = Utils.minMaxClipInt(cursorY + 1, 0, asciiWidth - 1);
            timer = System.currentTimeMillis();
            flash = true;
        }
        if (up.isClicked(gamepad.dpad_up)) {
            cursorY = Utils.minMaxClipInt(cursorY - 1, 0, asciiWidth - 1);
            timer = System.currentTimeMillis();
            flash = true;
        }
        if (left.isClicked(gamepad.dpad_left)) {
            cursorX = Utils.minMaxClipInt(cursorX + 1, 0, asciiLen - 1);
            timer = System.currentTimeMillis();
            flash = true;
        }
        if (right.isClicked(gamepad.dpad_right)) {
            cursorX = Utils.minMaxClipInt(cursorX - 1, 0, asciiLen - 1);
            timer = System.currentTimeMillis();
            flash = true;
        }

        if (a.isClicked(gamepad.a)) {
            sub[cursorX][cursorY] = !sub[cursorX][cursorY];
        }

        if (System.currentTimeMillis() - timer >= 500) {
            timer = System.currentTimeMillis();
            flash = !flash;
        }
    }
}
