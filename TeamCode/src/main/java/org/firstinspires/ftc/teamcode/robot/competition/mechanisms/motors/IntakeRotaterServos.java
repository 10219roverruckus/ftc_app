package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeRotaterServos {

    // instance variables
    public Servo intakeRotatorServoTop;
    public Servo intakeRotatorServoBottom;

    public LinearOpMode linearOp = null;

    // ############### NO CHANGES ABOVE THIS LINE ##################

    public double bottomServoLoweredPosition = .92; // made 2 more variables, 2 for each servo, for a total of 4 position variables
    public double bottomServoRaisedPosition = .25; //fine-tuned all variables
    public double topServoLoweredPosition = .96; //original positions- lowered: 0.9    raised: 0.23
    public double topServoRaisedPosition = .26;

    // ############## NO CHANGES BELOW THIS LINE ###################


    // constructor

    public IntakeRotaterServos(Servo iRT, Servo iRB) {
        intakeRotatorServoTop = iRT;
        intakeRotatorServoBottom = iRB;
    }

    public IntakeRotaterServos (Servo IRB) {
        intakeRotatorServoBottom = IRB;
    }

    // methods

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }


    public void  raisedRotater () {          // drops minerals into the lander
        intakeRotatorServoTop.setPosition(topServoRaisedPosition);
        intakeRotatorServoBottom.setPosition(bottomServoRaisedPosition);
    }

    public void loweredRotater () {
        intakeRotatorServoTop.setPosition(topServoLoweredPosition);      // keeps minerals in the little object
        intakeRotatorServoBottom.setPosition(bottomServoLoweredPosition);
    }

}



