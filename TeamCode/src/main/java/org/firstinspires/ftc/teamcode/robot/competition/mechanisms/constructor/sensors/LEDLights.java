package org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors;

import com.qualcomm.robotcore.hardware.Servo;



public class LEDLights {

    // instance variables
    double LEDRedPos = .6694;
    double LEDBluePos = .74150;
    double LEDGreenPos = .71409;
    double LEDPurplePos = .6609;
    double LEDWhiteBlinkingPos = .46249;
    double LEDYellowPos = .54949;


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
    public void LEDPurple () {
        LEDStrip.setPosition(LEDPurplePos);
    }
    public void LEDWhiteBlinking () {
        LEDStrip.setPosition(LEDWhiteBlinkingPos);
    }
    public void LEDYellow () {
        LEDStrip.setPosition(LEDYellowPos);
    }

}
