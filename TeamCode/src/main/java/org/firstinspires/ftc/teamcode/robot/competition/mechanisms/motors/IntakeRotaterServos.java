package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeRotaterServos {

    // instance variables
    public Servo intakeRotaterRight;
    public Servo intakeRotaterLeft;

    public LinearOpMode linearOp = null;

    public double leftRaisedPosition = 0;
    public double leftLoweredPosition = .94;

    public double rightRaisedPosition = .9;
    public double rightLoweredPostion = 0;

    // constructor

    public IntakeRotaterServos(Servo iRR, Servo iRL) {
        intakeRotaterLeft = iRL;
        intakeRotaterRight = iRR;
    }

    // methods

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }


    public void  raisedRotater () {          // drops minerals into the lander
        intakeRotaterRight.setPosition(rightRaisedPosition);
        intakeRotaterLeft.setPosition(leftRaisedPosition);
    }

    public void loweredRotater () {
        intakeRotaterLeft.setPosition(leftLoweredPosition);      // keeps minerals in the little object
        intakeRotaterRight.setPosition(rightLoweredPostion);
    }

}



