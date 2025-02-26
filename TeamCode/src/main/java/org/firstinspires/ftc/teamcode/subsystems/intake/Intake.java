package org.firstinspires.ftc.teamcode.subsystems.intake;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Utils;

public class Intake {
    public enum IntakeState{
        START_EXTEND,
        FINISH_EXTEND,
        ALIGN,
        LOWER,
        FINISH_GRAB,
        CONFIRM,
        RETRACT,
        HOLD,
        READY
    };
    private IntakeState intakeState = IntakeState.READY;

    private Robot robot;
    private EndAffector endAffector;

    private boolean close;

    private double targetLength, targetRotation;

    public static double intakeFlipUpAngle = -0.7;
    public static double intakeHoverAngle = -1.35;
    public static double intakeFlipGrabAngle = -1.7;
    public static double intakeFlipConfirmAngle = -1.3;
    public static double intakeFlipBackAngle = -0.1;

    public Intake(Robot robot){
        this.robot = robot;

        endAffector = new EndAffector(robot);
        close = false;
    }

    public void update(){
        endAffector.update();

        switch(intakeState){
            case START_EXTEND:
                endAffector.extendManual(0.0, intakeFlipUpAngle, targetRotation);
                endAffector.open();
                if(endAffector.flipInPosition()){
                    intakeState = IntakeState.FINISH_EXTEND;
                }
                break;
            case FINISH_EXTEND:
                endAffector.extendManual(targetLength, intakeHoverAngle, targetRotation);
                endAffector.open();
                if(endAffector.extendoInPosition()){
                    intakeState = IntakeState.ALIGN;
                    close = false;
                }
                break;
            case ALIGN:
                endAffector.extendManual(targetLength, intakeHoverAngle, targetRotation);
                endAffector.open();
                if(close && endAffector.rotInPosition()){
                    intakeState = IntakeState.LOWER;
                }
            case LOWER:
                endAffector.extendManual(targetLength, intakeFlipGrabAngle, targetRotation);
                endAffector.open();
                if(endAffector.flipInPosition()){
                    intakeState = IntakeState.FINISH_GRAB;
                }
                break;
            case FINISH_GRAB:
                endAffector.extendManual(targetLength, intakeFlipGrabAngle, targetRotation);
                endAffector.grab();
                if(endAffector.isClosed()){
                    intakeState = IntakeState.CONFIRM;
                }
                break;
            case CONFIRM:
                endAffector.extendManual(targetLength, intakeFlipConfirmAngle, targetRotation);
                endAffector.grab();
                if(!close){
                    intakeState = IntakeState.ALIGN;
                }
                break;
            case RETRACT:
                endAffector.extendManual(0.0, intakeFlipUpAngle, 0.0);
                if(close){
                    endAffector.grab();
                }else{
                    endAffector.open();
                }

                if(endAffector.inPosition()){
                    endAffector.extendManual(0.0, intakeFlipUpAngle, 0.0);
                    intakeState = close ? IntakeState.HOLD : IntakeState.READY;
                }
                break;
            case HOLD:
                endAffector.extendManual(0.0, intakeFlipUpAngle, 0.0);
                endAffector.grab();
                break;
            case READY:
                endAffector.extendManual(0.0, intakeFlipUpAngle, 0.0);
                endAffector.open();
                break;
        }
    }

    public void extend(){
        if(intakeState == IntakeState.READY || intakeState == IntakeState.HOLD){
            intakeState = IntakeState.START_EXTEND;
        }
    }

    public void release(){
        if(intakeState == IntakeState.HOLD){
            intakeState = IntakeState.READY;
            close = false;
        }
    }

    public void setClawRotation(double angle) {
        if (angle > 1.7) angle = -1.7;
        else if (angle < -1.7) angle = 1.7;
        targetRotation = angle;
    }

    public boolean isExtended(){
        return intakeState == IntakeState.ALIGN || intakeState == IntakeState.LOWER || intakeState == IntakeState.FINISH_GRAB || intakeState == IntakeState.CONFIRM;
    }

    public boolean hasSample(){
        return close;
    }

    public boolean grabFinished(){
        return intakeState == IntakeState.CONFIRM;
    }

    public void setIntakeExtension(double te, double tr){
        targetLength = Utils.minMaxClip(te, 0, 27.0);
        targetRotation = tr;
    }

    public double getIntakeTargetLength(){
        return targetLength;
    }

    public double getIntakeRotation(){
        return targetRotation;
    }

    public void grab(boolean g){
        close = g;
    }
}
