package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import android.support.annotation.Keep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class LanderServo {

    // instance variables
    public Servo landerServoL;
    public Servo landerServoR;
    public Servo transferGateServo;

    public LinearOpMode landerServoLinearOp = null;
    public LinearOpMode linearOp = null;

    public double LSScore = 1;
    public double LSCollect = 0;
    public double releaseMinerals = 1;
    public double keepMinerals = 0;

    // constructor

    public LanderServo ( Servo LSR, Servo LSL, Servo TGS) {
        LSR = landerServoL;
        LSL = landerServoR;
        TGS = transferGateServo;



        landerServoL.setDirection(Servo.Direction.FORWARD);
        landerServoR.setDirection(Servo.Direction.REVERSE);
    }

    // methods

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }

    public void landerServoScore () {          // drops minerals into the lander
        landerServoL.setPosition(LSScore);
        landerServoR.setPosition(LSScore);
    }

    public void landerServoCollect () {
        landerServoL.setPosition(LSCollect);      // keeps minerals in the little object
        landerServoR.setPosition(LSCollect);
    }

    public void releaseMinerals () {
        transferGateServo.setPosition(releaseMinerals);
    }

    public void keepMineralsIn () {
        transferGateServo.setPosition(keepMinerals);
    }
}



