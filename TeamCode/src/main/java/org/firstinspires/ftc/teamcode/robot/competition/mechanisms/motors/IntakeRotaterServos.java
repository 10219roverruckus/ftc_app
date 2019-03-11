package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeRotaterServos {

    // instance variables
    public Servo intakeRotaterRight;
    public Servo intakeRotaterLeft;

    public LinearOpMode linearOp = null;

    public double raisedPosition = 0;
    public double loweredPosition = .94;

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
        intakeRotaterRight.setPosition(raisedPosition);
        intakeRotaterLeft.setPosition(raisedPosition);
    }

    public void loweredRotater () {
        intakeRotaterLeft.setPosition(loweredPosition);      // keeps minerals in the little object
        intakeRotaterRight.setPosition(loweredPosition);
    }

}



