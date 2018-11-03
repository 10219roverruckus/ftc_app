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
    public final int RED_THRESHOLD = 30; //maybe a higher threshold (12.5)
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



   // LiftMotor myLiftMotor;
//    TeamMarker myTeamMarker;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }

    //in future, will pass Camera as Parameter
    public MecanumMineralMiner() {
    }
// both crater and depot
    public void findingMineral() {
//        if (cameraGoldLocation < 280) {
//            goldPosition = GoldPosition.LEFT;
//        }
//        else if (cameraGoldLocation > 500) {
//            goldPosition = GoldPosition.RIGHT;
//        }
//        else {
//            goldPosition = GoldPosition.MIDDLE;
//        }
        goldPosition = goldPosition.MIDDLE;
    }

    public void findingMineralCamera (double cameraGoldLocation) {
        if (cameraGoldLocation < 300 && cameraGoldLocation > 1) {
            goldPosition = GoldPosition.MIDDLE;
        }
        else if (cameraGoldLocation > 300) {
            goldPosition = GoldPosition.RIGHT;
        }
        else {
            goldPosition = GoldPosition.LEFT;
        }
    }

//both crater and depot
    public void driveMineral(GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor) {

        linearOp.telemetry.addData("MINERAL", goldPosition);
        linearOp.telemetry.update();
        myLiftMotor.extendLiftMotorFullyEncoders(); //distance sensor

        myMechDrive.strafeRight(.3,.3);
        myMechDrive.driveForward(.3, .3); //DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        switch (goldPosition) {  //ANGLES SELF AND GOES TOWARD GOLD MINERAL
            case LEFT:
//                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
//                linearOp.telemetry.update();
//                linearOp.sleep(2000);
                myGyro.gyroOrientMecanum(34, myMechDrive);
                myMechDrive.driveForward(SPD_DRIVE_MED, 1.8);
                break;

            case MIDDLE:
//                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
//                linearOp.telemetry.update();
//                linearOp.sleep(2000);
                myGyro.gyroOrientMecanum(7, myMechDrive);
                myMechDrive.driveForward(SPD_DRIVE_MED, 1.7);
                break;

            case RIGHT:
//                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
//                linearOp.telemetry.update();
//                linearOp.sleep(2000);
                myGyro.gyroOrientMecanum(-14, myMechDrive);
                myMechDrive.driveForward(SPD_DRIVE_MED, 1.8);
                break;
        }
    }
// crater
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
            myMechDrive.setMotorSpeeds(-SPD_DRIVE_MED);
            //myMechDrive.setMotorSpeeds(SPD_DRIVE_MED);
            linearOp.idle();
        }
        myMechDrive.stopMotors();
        //DRIVE FUNCTION DOESN'T HAVE A STOP.MOTORS IN IT
 //       myMechDrive.driveForward(SPD_DRIVE_MED, .3);
        myGyro.gyroOrientMecanum(72, myMechDrive);  //orients self with red tape so parallel to tape.

        //MAY NEED TO BE LESS THAN 90 DEGRESS SO ROBOT DOES NOT HIT THE LANDER LEG!



        //IN THEORY, THE DRIVE DISTANCE SHOULD BE THE SAME OR CLOSE FOR L / M / R
        //DRIVES TO WALL
        switch (goldPosition) {  //DRIVE TO WALL
            case LEFT:
                myMechDrive.driveForward(SPD_DRIVE_MED, 3);
                break;
            case MIDDLE:
                myMechDrive.driveForward(SPD_DRIVE_MED, 3.5);
                break;
            case RIGHT:
                myMechDrive.driveForward(SPD_DRIVE_MED, 4.0);
                break;
        }
    }
//crater
    public void wallToDepot(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance, TeamMarker myTeamMarker) {
        myGyro.gyroOrientMecanum(127, myMechDrive); // 130 last night
        myMechDrive.setMotorPowerStrafeRight(.3);
        linearOp.sleep(1000);
        myMechDrive.stopMotors();
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
            myMechDrive.setMotorSpeeds(SPD_DRIVE_MED);
            linearOp.idle();
        }
        myMechDrive.stopMotors();
        myGyro.gyroOrientMecanum(170, myMechDrive);
        myMechDrive.stopMotors();
        myMechDrive.strafeLeft(.2, .2);

        myTeamMarker.teamMarkerArmOutside();
        linearOp.sleep(1250);
        myTeamMarker.teamMarkerArmRaised();
        linearOp.sleep(500);
        myGyro.gyroOrientMecanum(128, myMechDrive);
        myMechDrive.stopMotors();
        myMechDrive.strafeRight(.2, .2);
        myGyro.gyroOrientMecanum(128, myMechDrive);
        myMechDrive.setMotorPowerStrafeRight(.3);
        linearOp.sleep(500);
        myMechDrive.driveBackward(SPD_DRIVE_MED, 6.2);
    }

// depot
    public void mineralToDepot (GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance, TeamMarker myTeamMarker) {
        switch (goldPosition) {
            case LEFT:
                myGyro.gyroOrientMecanum(-28, myMechDrive);  //rotates in toward depot

            case MIDDLE:
                myGyro.gyroOrientMecanum(4, myMechDrive); //drive forward towards depo just a little bit

            case RIGHT:
                myGyro.gyroOrientMecanum(31, myMechDrive);  //rotates in toward depot
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
            myMechDrive.setMotorSpeeds(SPD_DRIVE_MED);
            linearOp.idle();
        }

//
//        myGyro.gyroOrientMecanum(42, myMechDrive);
//
//        myTeamMarker.teamMarkerArmOutside ();
//        myTeamMarker.teamMarkerArmRaised();
//
//        myGyro.gyroOrientMecanum(133.5, myMechDrive);




    }
//depot
    public void depotToCrater (GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {

        myMechDrive.driveBackward(SPD_DRIVE_MED, 3);
        myGyro.gyroOrientMecanum(45, myMechDrive);
        myMechDrive.driveBackward(SPD_DRIVE_MED, 3.5);

    }
}

