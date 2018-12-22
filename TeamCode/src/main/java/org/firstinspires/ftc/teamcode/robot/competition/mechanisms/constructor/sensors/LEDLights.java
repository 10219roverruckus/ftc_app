package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors;

import com.qualcomm.robotcore.hardware.Servo;



public class LEDLights {

    // instance variables
    int LEDRedPos = 1000;
    int LEDBluePos = 1000;
    int LEDGreenPos = 1000;


    public Servo LEDStrip; // strip of LED

    // constructer
    public LEDLights (Servo Strip) {
        LEDStrip = Strip;
    }

    // methods

    public void LEDred () {
        LEDStrip.setPosition(LEDRedPos);
    }
    public void LEDblue () {
        LEDStrip.setPosition(LEDBluePos);
    }
    public void LEDgreen () {
        LEDStrip.setPosition(LEDGreenPos);
    }
}
