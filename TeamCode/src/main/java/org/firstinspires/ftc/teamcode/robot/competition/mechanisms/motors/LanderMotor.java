package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class LanderMotor {



    // instance variables
    public DcMotor landerMotor;

    public LinearOpMode landerMotorLinearOp = null;
    public LinearOpMode linearOp = null;

    // constructor
    public LanderMotor ( DcMotor LM) {
        LM = landerMotor;

        landerMotor.setDirection(DcMotor.Direction.FORWARD);
        landerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    // methods

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }
    public void setLanderMotorLinearOp (LinearOpMode Op) {
        landerMotorLinearOp = Op;
    }



    public void extendLanderMotor (double motorPower) {
        landerMotor.setPower(Math.abs(motorPower));
    }

    public void retractLanderMotor (double motorPower) {

        landerMotor.setPower(-Math.abs(motorPower));

    }

    public void stopLanderMotor () {
        landerMotor.setPower(0);
    }
}
