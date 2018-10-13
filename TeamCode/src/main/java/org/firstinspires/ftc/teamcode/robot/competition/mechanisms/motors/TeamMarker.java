package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.hardware.Servo;

public class TeamMarker {

    // instance variables
    private Servo teamMarkerArm; // actually the arm
    private double tMArmRaised = 1; // arm raised position
    private double tMArmLowered = 0; // arm lowered position


    // constructors
    public TeamMarker(Servo tMarkArm) {
        teamMarkerArm = tMarkArm;
    }


    //methods
    public void teamMarkerArmRaised () {            // raises arm after you put the team marker down
        teamMarkerArm.setPosition(tMArmRaised);     // or when you are driing over to the square to place team marker
    }

    public void teamMarkerArmLowered () {          // lowers arm to put team marker in square
        teamMarkerArm.setPosition(tMArmLowered);
    }



}
