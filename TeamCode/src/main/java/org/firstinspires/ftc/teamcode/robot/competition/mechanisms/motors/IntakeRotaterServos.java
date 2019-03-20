package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeRotaterServos {

    // instance variables
    public Servo intakeRotaterServo;
//    public Servo intakeRotaterLeft;

    public LinearOpMode linearOp = null;

    public double loweredPosition = 0;

    public double raisedPosition = .9;


    // constructor

    public IntakeRotaterServos(Servo iRL) {
        intakeRotaterServo = iRL;
//        intakeRotaterRight = iRR;
    }

    // methods

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }


    public void  raisedRotater () {          // drops minerals into the lander
        intakeRotaterServo.setPosition(raisedPosition);
    }

    public void loweredRotater () {
        intakeRotaterServo.setPosition(loweredPosition);      // keeps minerals in the little object
    }

}



