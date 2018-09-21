package org.firstinspires.ftc.teamcode.robot.old.classes.sub;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by blake_shafer on 10/16/17.
 */

public class relicArm {

    public DcMotor relicArmLift;
    public DcMotor relicArmExtension;
    public Servo relicGrabber;

    double openPosition = .21;
    double closePosition = 1;
    double almostClosePosition = 0.925;
    double grabberPosition;

    public relicArm(DcMotor rAL, DcMotor rAE, Servo rG) {
        relicArmLift = rAL;
        relicArmExtension = rAE;
        relicGrabber = rG;

        relicArmLift.setDirection(DcMotor.Direction.REVERSE);
        relicArmLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicArmLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicArmLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        relicArmExtension.setDirection(DcMotor.Direction.FORWARD);
        relicArmExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicArmExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicArmExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLiftPower(double liftPower) {
        relicArmLift.setPower(liftPower);
    }

    public void setExtensionPower(double extensionPower) {
        relicArmExtension.setPower(extensionPower);
    }

    public void relicGrabberOpen() {
        relicGrabber.setPosition(openPosition);
    }

    public void relicGrabberClose() {
        relicGrabber.setPosition(closePosition);
    }

    public double getRelicGrabberPosition() {
        grabberPosition = relicGrabber.getPosition();
        return grabberPosition;
    }

    public void relicGrabberAlmostClose() {
        relicGrabber.setPosition(almostClosePosition);
    }

    public void setRelicGrabberPosition(double position) {
        relicGrabber.setPosition(position);
    }
}