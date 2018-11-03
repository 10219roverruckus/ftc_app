package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class TeamMarker {

    // instance variables
    public Servo teamMarkerArm; // actually the arm
    public double tMArmRaised = .76; // arm raised position
    public double tMArmOutside = .25 ; // arm lowered position
    public double tMArmInside = 1;
    public LinearOpMode linearOp = null;

    // constructors
    public TeamMarker(Servo tMarkArm) {

        teamMarkerArm = tMarkArm;
    }

    public void setLinearOp (LinearOpMode Op) {
        linearOp = Op;
    }


    //methods
    public void teamMarkerArmRaised () {            // raises arm after you put the team marker down
        teamMarkerArm.setPosition(tMArmRaised);     // or when you are driing over to the square to place team marker
    }

    public void teamMarkerArmOutside () {          // lowers arm to put team marker in square
        teamMarkerArm.setPosition(tMArmOutside);
    }

    public void teamMarkerInside () {
        teamMarkerArm.setPosition(tMArmInside);
    }
}
