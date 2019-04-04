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

    public double LSScore = .5;         // Scoring Postion
    public double LSTravel = .72;       // Travel Position
    public double LSCollect = .94;      // Collecting Upright Position


    public double CloseGate = .146; //was .184
    public double OpenGate = .55;  //was .41 = .36 difference

    // constructor

    public LanderServo (Servo LSL) {
        landerServoR = LSL;
//        transferGateServo = TGS;
    }

    // methods

    public void setLinearOp (LinearOpMode Op) {

        linearOp = Op;
    }

    public void landerServoScore () {          // Scoring and Dropping minerals into the lander

        landerServoR.setPosition(LSScore);
    }

    public void landerServoTravel() {           // Starting and Travel Position of the Dump

        landerServoR.setPosition(LSTravel);
    }

    public void landerServoCollect () {         // Collecting & Upright to keep minerals in the little object

        landerServoR.setPosition(LSCollect);
    }



//    public void releaseMinerals () {
//        transferGateServo.setPosition(OpenGate);
//    }

//    public void keepMineralsIn () {
//        transferGateServo.setPosition(CloseGate);
//    }
}



