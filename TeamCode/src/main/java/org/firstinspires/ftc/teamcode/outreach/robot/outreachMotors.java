package org.firstinspires.ftc.teamcode.outreach.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class outreachMotors {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    public outreachMotors (DcMotor lm, DcMotor rm) {
        leftMotor = lm;
        rightMotor = rm;

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive (double leftY, double rightY) {
        leftY = Range.clip(leftY,-1,+1);
        rightY = Range.clip(rightY, -1 ,1);
        if (leftY < -.1 || leftY > .1) {
            leftMotor.setPower(leftY);
        }
        else {
            leftMotor.setPower(0);
        }
        if (rightY < -.1 || rightY > .1) {
            rightMotor.setPower(rightY);
        }
        else {
            rightMotor.setPower(0);
        }
    }
}
