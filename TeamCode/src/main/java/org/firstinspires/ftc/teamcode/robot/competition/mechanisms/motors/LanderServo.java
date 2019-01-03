package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import android.support.annotation.Keep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class LanderServo {

    // instance variables
    public Servo landerServoR;
    public Servo transferGateServo;

    public LinearOpMode linearOp = null;

    public double LSScore = .56;
    public double LSCollect = 1.0;
    public double CloseGate = .05;
    public double OpenGate = .41;

    // constructor

    public LanderServo (Servo LSL, Servo TGS) {
        landerServoR = LSL;
        transferGateServo = TGS;
    }

    // methods

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }

    public void landerServoScore () {          // drops minerals into the lander
        landerServoR.setPosition(LSScore);
    }

    public void landerServoCollect () {
        landerServoR.setPosition(LSCollect);      // keeps minerals in the little object
    }

    public void releaseMinerals () {
        transferGateServo.setPosition(OpenGate);
    }

    public void keepMineralsIn () {
        transferGateServo.setPosition(CloseGate);
    }
}



