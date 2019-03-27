package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeRotaterServos {

    // instance variables
    public Servo intakeRotatorServoTop;
    public Servo intakeRotatorServoBottom;

    public LinearOpMode linearOp = null;

    // ############### NO CHANGES ABOVE THIS LINE ##################

    public double loweredPosition = .9;
    public double raisedPosition = 0.23;

    // ############## NO CHANGES BELOW THIS LINE ###################


    // constructor

    public IntakeRotaterServos(Servo iRT, Servo iRB) {
        intakeRotatorServoTop = iRT;
        intakeRotatorServoBottom = iRB;
    }

    // methods

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }


    public void  raisedRotater () {          // drops minerals into the lander
        intakeRotatorServoTop.setPosition(raisedPosition);
        intakeRotatorServoBottom.setPosition(raisedPosition);
    }

    public void loweredRotater () {
        intakeRotatorServoTop.setPosition(loweredPosition);      // keeps minerals in the little object
        intakeRotatorServoBottom.setPosition(loweredPosition);
    }

}



