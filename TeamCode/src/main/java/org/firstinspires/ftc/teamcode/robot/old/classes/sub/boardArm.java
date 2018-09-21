package org.firstinspires.ftc.teamcode.robot.old.classes.sub;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by blake_shafer on 11/22/17.
 */

public class boardArm {

    public DcMotor boardArm;

    public boardArm(DcMotor bA) {
        boardArm = bA;
        boardArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        boardArm.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void boardArmUp() {
        boardArm.setPower(0.8);
    }

    public void boardArmDown() {
        boardArm.setPower(-0.8);
    }

    public void boardArmStop() {
        boardArm.setPower(0);
    }
}
