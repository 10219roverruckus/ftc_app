package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.TeamMarker;


public class MecanumMineralMiner {

    public GoldPosition goldPosition = null;
    public LinearOpMode linearOp = null;
    public final int RED_THRESHOLD = 15; //maybe a higher threshold (12.5)
    public final int BLUE_THRESHOLD = 100; //maybe a higher threshold (12.5)
    float hsvValues[] = {0F, 0F, 0F};



    // created constant variables that are used for speed (different setting)
    final double SPD_DRIVE_LOW = .20;     //Lowest speed
    final double SPD_DRIVE_MED = .4;      //Default is  SPD_MED
    final double SPD_DRIVE_HIGH = .75;
    final double SPD_DRIVE_MAX = 1.0;
    final double SPD_ARM_MED = .5;
    final long sleepTime = 0;
    final float values[] = hsvValues;

    final double SCALE_FACTOR = 255;



    LiftMotor myLiftMotor;
    TeamMarker myTeamMarker;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }

    //in future, will pass Camera as Parameter
    public MecanumMineralMiner() {
    }

    public void findingMineral() {
//        if (detector.getXPosition() < 280) {
//            goldPosition = GoldPosition.LEFT;
//        }
//        else if (detector.getXPosition() > 500) {
//            goldPosition = GoldPosition.RIGHT;
//        }
//        else {
//            goldPosition = GoldPosition.MIDDLE;
//        }

        goldPosition = GoldPosition.RIGHT;
    }

    public void driveMineral(GyroCompetition myGyro, MecanumDrive myMechDrive) {

        myLiftMotor.extendLiftMotorFully(); //distance sensor

        myMechDrive.strafeRight(.3,.3);
        myMechDrive.driveForward(.3, .5); //DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        switch (goldPosition) {  //ANGLES SELF AND GOES TOWARD GOLD MINERAL
            case LEFT:
//                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
//                linearOp.telemetry.update();
//                linearOp.sleep(2000);
                myGyro.gyroOrientMecanum(34, myMechDrive);
                myMechDrive.driveForward(SPD_DRIVE_MED, 3.4);
                break;

            case MIDDLE:
//                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
//                linearOp.telemetry.update();
//                linearOp.sleep(2000);
                myGyro.gyroOrientMecanum(7, myMechDrive);
                myMechDrive.driveForward(SPD_DRIVE_MED, 2.8);
                break;

            case RIGHT:
//                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
//                linearOp.telemetry.update();
//                linearOp.sleep(2000);
                myGyro.gyroOrientMecanum(-11, myMechDrive);
                myMechDrive.driveForward(SPD_DRIVE_MED, 2.7);
                break;
        }
    }

    public void craterMineralToWall(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {
        //Drives back to tape
        // SHOULD BE ABLE TO USE FOR DETECTING EITHER COLOR.
        myMechDrive.stopMotors();
        Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
                (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
                (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        while (hsvValues[0] > RED_THRESHOLD && hsvValues[0] < BLUE_THRESHOLD) {
            Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            myMechDrive.driveBackward(SPD_DRIVE_MED, -.4);
            linearOp.idle();
        }
        myMechDrive.stopMotors();
        //DRIVE FUNCTION DOESN'T HAVE A STOP.MOTORS IN IT
        myMechDrive.driveForward(SPD_DRIVE_MED, .4);
        myGyro.gyroOrientMecanum(87, myMechDrive);  //orients self with red tape so parallel to tape.
        //
        //MAY NEED TO BE LESS THAN 90 DEGRESS SO ROBOT DOES NOT HIT THE LANDER LEG!
        //


        //IN THEORY, THE DRIVE DISTANCE SHOULD BE THE SAME OR CLOSE FOR L / M / R
        //DRIVES TO WALL
        switch (goldPosition) {  //DRIVE TO WALL
            case LEFT:
                myMechDrive.driveForward(SPD_DRIVE_MED, 6);
                break;
            case MIDDLE:
                myMechDrive.driveForward(SPD_DRIVE_MED, 6);
                break;
            case RIGHT:
                myMechDrive.driveForward(SPD_DRIVE_MED, 6.5);
                break;
        }
    }

    public void wallToDepot(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {
        myGyro.gyroOrientMecanum(130, myMechDrive);
        myMechDrive.driveForward(SPD_DRIVE_MED, 3);
        Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
                (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
                (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        while (hsvValues[0] > RED_THRESHOLD && hsvValues[0] < BLUE_THRESHOLD) {
            Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            myMechDrive.driveForward(SPD_DRIVE_MED, .4);
            linearOp.idle();
        }
        myMechDrive.stopMotors();
        myMechDrive.driveBackward(SPD_DRIVE_MED, 6);
    }


    public void mineralToDepot (GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {
        switch (goldPosition) {
            case LEFT:
                myGyro.gyroOrientMecanum(-18, myMechDrive);  //rotates in toward depot

            case MIDDLE:
                myMechDrive.driveForward(SPD_DRIVE_MED,.4); //drive forward towards depo just a little bit

            case RIGHT:
                myGyro.gyroOrientMecanum(18, myMechDrive);  //rotates in toward depot
        }

        Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
                (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
                (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        while (hsvValues[0] > RED_THRESHOLD && hsvValues[0] < BLUE_THRESHOLD) {
            Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            myMechDrive.driveForward(SPD_DRIVE_MED, .4);
            linearOp.idle();
        }


        myGyro.gyroOrientMecanum(45, myMechDrive);
        myMechDrive.strafeRight(.2,.4);

        myTeamMarker.teamMarkerArmOutside ();
        myTeamMarker.teamMarkerArmRaised();




    }

    public void depotToCrater (GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {

        myMechDrive.driveBackward(SPD_DRIVE_MED, 3);
        myGyro.gyroOrientMecanum(45, myMechDrive);
        myMechDrive.driveBackward(SPD_DRIVE_MED, 3.5);

    }
}

