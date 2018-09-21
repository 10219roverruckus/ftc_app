package org.firstinspires.ftc.teamcode.robot.old.classes.sub;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

/**
 * Created by johnduval on 10/7/17.
 */

public class colorSensorArm {

    int redThreshold = 2;
    int blueThreshold = 2;
    int colorLoop = 0;
    //double restPosition = 0.2;
    double upPositionPause1 = .40;
    double upPosition = 0.16;
    double downPositionPause1 = 0.70;
    double downPositionPause2 = 0.88;
    double downPositionFinal = 0.94;
    int colorArmPause = 1000;
    double colorArmIncrementAmount = .01;
    int colorArmIncrementTimeMSdown = 12; //will take aprox. 1 second to lower color sensor arm .
    int colorArmIncrementTimeMSup = 16; //will take aprox. 1 second to lower color sensor arm .

    double colorSensorArmReadingPosition = 0.452; //rotate arm
    double colorSensorArmRestingPosition = 0.45;  //rotate arm

    public Servo colorSensorArm;
    public ColorSensor colorSensor;
    public Servo colorSensorArmRotate;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;
    float hsvValues[] = {0F, 0F, 0F};

    //mechDriveAuto myMechDrive;

    public colorSensorArm(Servo cSA, ColorSensor cS, Servo cSAR) {
        colorSensorArm = cSA;
        colorSensorArmRotate = cSAR;
        colorSensor = cS;

        //  MODERN ROBOTICS COLOR SENSOR CODE
        // hsvValues is an array that will hold the hue, saturation, and value information.
        //float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;



        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);
    }

    public void colorSensorArmUp() {
        colorSensorArm.setPosition(upPosition);
    }

    public void colorSensorArmUpSlow () throws InterruptedException {
        double colorArmIncrementPosition = upPositionPause1;
        colorSensorArm.setPosition(colorArmIncrementPosition);
        sleep(colorArmPause);
        colorRotateResting();
        while (colorArmIncrementPosition >=  upPosition) {
            colorArmIncrementPosition = colorArmIncrementPosition - colorArmIncrementAmount;
            colorSensorArm.setPosition(colorArmIncrementPosition);
            sleep(colorArmIncrementTimeMSup);
        }
    }

    public void colorSensorArmDown() throws InterruptedException {
        colorSensorArm.setPosition(downPositionPause1);
        sleep(colorArmPause);
        colorSensorArm.setPosition(downPositionPause2);
        sleep(colorArmPause);
        colorSensorArm.setPosition(downPositionFinal);
    }

    public void colorSensorArmDownSlow() throws InterruptedException {
        double colorArmIncrementPosition = downPositionPause1;
        colorSensorArm.setPosition(colorArmIncrementPosition);
        colorRotateReading();
        sleep(colorArmPause);
        while (colorArmIncrementPosition <=  downPositionFinal) {
            colorArmIncrementPosition = colorArmIncrementPosition + colorArmIncrementAmount;
            colorSensorArm.setPosition(colorArmIncrementPosition);
            sleep(colorArmIncrementTimeMSdown);
        }
    }

    public void colorRotateResting () {
        colorSensorArmRotate.setPosition(colorSensorArmRestingPosition);
    }

    public void colorRotateReading () {
        colorSensorArmRotate.setPosition(colorSensorArmReadingPosition);
    }

    public void colorRotateClockwise () {
        colorSensorArmRotate.setPosition(0.95);
    }

    public void colorRotateCounterClockwise () {
        colorSensorArmRotate.setPosition(0);
    }

    public int colorJewel() throws InterruptedException {
        colorLoop++;
        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR), hsvValues);
        //gives sensor time to be accurate
        //1 == red
        //2 == blue
        //3 = none detected
        if (colorSensor.red() > colorSensor.blue()) {
            return 1;
        }
        else if (colorSensor.blue() > colorSensor.red()) {
            return 2;
        }
        else {
            return 3;
        }
    }
}