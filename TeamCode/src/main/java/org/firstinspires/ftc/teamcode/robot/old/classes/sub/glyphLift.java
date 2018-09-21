package org.firstinspires.ftc.teamcode.robot.old.classes.sub;

import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Thread.sleep;

/**
 * Created by blake_shafer on 10/16/17.
 */

public class glyphLift {

    public DcMotor glyphLift;
    private double autRaisePower = 1.0;
    private int autRaiseTime = 600;
    private int autLowerTime = 200;
    int position;

    public glyphLift(DcMotor gL) {
        glyphLift = gL;

        glyphLift.setDirection(DcMotor.Direction.REVERSE);
        glyphLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        glyphLift.setPower(power);
    }

    public int getCurrentPosition() {
        position = glyphLift.getCurrentPosition();
        return position;
    }

    public void raiseGlyphLiftAutMode () throws InterruptedException {
        glyphLift.setPower(autRaisePower);
        sleep(autRaiseTime);
        glyphLift.setPower(0);
    }

    public void lowerGlyphLiftAutMode () throws InterruptedException {
        glyphLift.setPower(-autRaisePower);
        sleep(autLowerTime);
        glyphLift.setPower(0);
    }
}