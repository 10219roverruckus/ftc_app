package org.firstinspires.ftc.teamcode.robot.competition.oldClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.autonomous.GoldPosition;


public class MecanumMineralMinerAll {

    public GoldPosition goldPosition = null;
    public LinearOpMode linearOp = null;
    public final int RED_THRESHOLD = 30;                //maybe a higher threshold (12.5)
    public final int BLUE_THRESHOLD = 100;              //maybe a higher threshold (12.5)
    float hsvValues[] = {0F, 0F, 0F};

    // created constant variables that are used for speed (different setting)

    final double SPD_DRIVE_LOW = .20;                  //Lowest speed
    final double SPD_DRIVE_MED = .4;                   //Default is  SPD_MED
    final double SPD_DRIVE_HIGH = .75;
    final double SPD_DRIVE_MAX = 1.0;
    final double SPD_ARM_MED = .5;
    final long sleepTime = 0;

    // variables and constants used by color sensor

    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }
        //in future, will pass Camera as Parameter


    public MecanumMineralMinerAll() {
    }


    public void findingMineralCamera(double cameraGoldLocation) {

         //find location of the mineral using camera

//        if (cameraGoldLocation < 300 && cameraGoldLocation > 1) {
//            goldPosition = GoldPosition.MIDDLE;                           //commented out while camera does not work
//        } else if (cameraGoldLocation > 300) {                            //program works perfectly DO NOT CHANGE THE CODE
//            goldPosition = GoldPosition.RIGHT;
//        } else {
//            goldPosition = GoldPosition.LEFT;
//        }
        goldPosition = GoldPosition.MIDDLE;
    }
}