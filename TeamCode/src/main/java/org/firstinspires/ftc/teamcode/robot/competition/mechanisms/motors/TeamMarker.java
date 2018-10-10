package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors;

import com.qualcomm.robotcore.hardware.Servo;

public class TeamMarker {

    // instance variables
    private Servo teamMarkerArm; // actually arm
    private double tMArmRaised = 1; // arm raised
    private double tMArmLowered = 0; // arm lowered


    // constructors
    public TeamMarker(Servo tMarkArm) {
        teamMarkerArm = tMarkArm;
    }


    //methods
    public void teamMarkerArmRaised () {
        teamMarkerArm.setPosition(tMArmRaised);
    }

    public void teamMarkerArmLowered () {
        teamMarkerArm.setPosition(tMArmLowered);
    }



}
