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
                myGyro.gyroOrientMecanum(36, myMechDrive);  // different (34)
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 2.15); // different (1.8)
                break;

            case MIDDLE:
//                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
//                linearOp.telemetry.update();
//                linearOp.sleep(2000);
                myGyro.gyroOrientMecanum(4, myMechDrive); //turning too much towrads the right EA
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 1.7); // different (1.7)
                break;

            case RIGHT:
//                linearOp.telemetry.addData("MINERAL POSITION: ", goldPosition);
//                linearOp.telemetry.update();
//                linearOp.sleep(2000);
                myGyro.gyroOrientMecanum(-14, myMechDrive);
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 2); // different (1.8)
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
        myGyro.gyroOrientMecanum(74, myMechDrive);
        myMechDrive.stopMotors();  //orients self with red tape so parallel to tape.
        //different (72)

        //MAY NEED TO BE LESS THAN 90 DEGRESS SO ROBOT DOES NOT HIT THE LANDER LEG!



        //IN THEORY, THE DRIVE DISTANCE SHOULD BE THE SAME OR CLOSE FOR L / M / R
        //DRIVES TO WALL
        switch (goldPosition) {  //DRIVE TO WALL
            case LEFT:
                myMechDrive.driveForward(SPD_DRIVE_MED, 3.2); // different but I think it is better
                break;
            case MIDDLE:
                myMechDrive.driveForward(SPD_DRIVE_MED, 3.7); // different but I think it is better
                break;
            case RIGHT:
                myMechDrive.driveForward(SPD_DRIVE_MED, 4.2); // different but I think it is better
                break;
        }
    }
//crater
    public void wallToDepot(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance, TeamMarker myTeamMarker) {
        myGyro.gyroOrientMecanum(137, myMechDrive); // 127 but it did not mske sense because the other angle was 138 and they were the same angle and then it was too big so now it is 137
        myMechDrive.stopMotors();
        myMechDrive.setMotorPowerStrafeRight(.3);
        linearOp.sleep(1500); //was 1000 but it needed to be longer
        myMechDrive.stopMotors();

        //myMechDrive.driveForward(SPD_DRIVE_MED, 1.5); was about to try to gyro away from the wall and then back to the wall around the seam
        //myMechDrive.stopMotors();
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
        myMechDrive.strafeLeft(.2, .3); // .2 , .2  // different but I think it is better

        myTeamMarker.teamMarkerArmOutside();
        linearOp.sleep(1250);
        myTeamMarker.teamMarkerArmRaised();
        linearOp.sleep(500);

        myMechDrive.strafeLeft(SPD_DRIVE_LOW,.2);
        myMechDrive.driveBackward(SPD_DRIVE_LOW,.7);

        myGyro.gyroOrientMecanum(138, myMechDrive); // was 128 but it was to small of the turn
        myMechDrive.stopMotors();

        myMechDrive.setMotorPowerStrafeRight(.3);  // staffe into wall
        linearOp.sleep(1000);

        myMechDrive.driveBackward(SPD_DRIVE_MED, 2.0); //Drive to plexiglass seem
        myGyro.gyroOrientMecanum(139, myMechDrive);   // Gyo correction for plexiglass  was 137
        myMechDrive.stopMotors();
        myMechDrive.driveBackward(SPD_DRIVE_MED, 3.3); //Drive past plexiglass seem

    }

// depot
    public void mineralToDepot (GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance, TeamMarker myTeamMarker) {
        switch (goldPosition) {
            case LEFT:   // not working
                myMechDrive.driveForward(SPD_DRIVE_LOW,1);
                myMechDrive.driveBackward(SPD_DRIVE_LOW,.2);

                myGyro.gyroOrientMecanum(-28, myMechDrive);  //rotates in toward depot
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED,2.2);

                myGyro.gyroOrientMecanum(42, myMechDrive); // turn was 42 (perfect)
                myMechDrive.stopMotors();

                myMechDrive.strafeLeft(.2,.3); // rotations .3 (prefect)

                myTeamMarker.teamMarkerArmOutside ();
                linearOp.sleep(1000);
                myTeamMarker.teamMarkerArmRaised();

                myGyro.gyroOrientMecanum(130, myMechDrive);
                myMechDrive.stopMotors();
                break;


            case MIDDLE: // duplicating code
                myGyro.gyroOrientMecanum(0, myMechDrive); //drive forward towards depo just a little bit
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED,2.1); // push mineral into depot for extra point
                myMechDrive.driveBackward(SPD_DRIVE_LOW, .2); // back up clear mineral

                myGyro.gyroOrientMecanum(60, myMechDrive); // turn to drop team marker
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED,.3); // inch forward a little bit

                // myMechDrive.strafeLeft(.2,.3); // strafe

                myTeamMarker.teamMarkerArmOutside (); // drop maker
                linearOp.sleep(1000);
                myTeamMarker.teamMarkerArmRaised(); // lift servo arm

                myGyro.gyroOrientMecanum(130, myMechDrive); // orient towards depot
                myMechDrive.stopMotors();
                break;

            case RIGHT: // working
                myMechDrive.driveForward(SPD_DRIVE_LOW,1.1); // pushimg mineral to clear middle mineral
                myMechDrive.driveBackward(SPD_DRIVE_LOW,.2);  // backup so turn does not hit mineral
                myGyro.gyroOrientMecanum(32, myMechDrive);  //rotates in toward depot
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED,2.1);

                myGyro.gyroOrientMecanum(60, myMechDrive);
                myMechDrive.stopMotors();

                myMechDrive.strafeLeft(.2,.3);

                myTeamMarker.teamMarkerArmOutside ();
                linearOp.sleep(1000);
                myTeamMarker.teamMarkerArmRaised();

                myGyro.gyroOrientMecanum(130, myMechDrive);
                myMechDrive.stopMotors();
                break;
        }

//
//        Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
//                (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
//                (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
//                hsvValues);
//        while (hsvValues[0] > RED_THRESHOLD && hsvValues[0] < BLUE_THRESHOLD) {
//            Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
//                    (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
//                    (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
//                    hsvValues);
//            myMechDrive.setMotorSpeeds(SPD_DRIVE_LOW);
//            linearOp.idle();
//        }

        myMechDrive.stopMotors();





    }
//depot
    public void depotToCrater (GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {

        switch (goldPosition) {
            case LEFT: // not working

                myMechDrive.setMotorPowerStrafeRight(.5);
                linearOp.sleep(1500);

                myMechDrive.driveForward(SPD_DRIVE_MED, 2);
                myGyro.gyroOrientMecanum(130, myMechDrive);
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3);
                break;

            case MIDDLE:
                myMechDrive.setMotorPowerStrafeRight(.5);
                linearOp.sleep(2000);

                myMechDrive.driveForward(SPD_DRIVE_MED, 2);
                myGyro.gyroOrientMecanum(130, myMechDrive);
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3.2);
                break;

            case RIGHT: // working
                myMechDrive.setMotorPowerStrafeRight(.5);
                linearOp.sleep(1500);

                myMechDrive.driveForward(SPD_DRIVE_MED, 2);
                myGyro.gyroOrientMecanum(130, myMechDrive);
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3.4);
                break;
        }

        //crater
//        public void craterFromDepotToCrater(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {
//            switch (goldPosition) {
//                case LEFT: // not working
//
//                    myMechDrive.setMotorPowerStrafeRight(.5);
//                    linearOp.sleep(1500);
//
//                    myMechDrive.driveBackward(SPD_DRIVE_MED, 2);
//                    myGyro.gyroOrientMecanum(134, myMechDrive);
//                    myMechDrive.stopMotors();
//
//                    myMechDrive.driveForward(SPD_DRIVE_MED, 3);
//                    break;
//
//                case MIDDLE:
//                    myMechDrive.setMotorPowerStrafeRight(.5);
//                    linearOp.sleep(2000);
//
//                    myMechDrive.driveBackward(SPD_DRIVE_MED, 2);
//                    myGyro.gyroOrientMecanum(134, myMechDrive);
//                    myMechDrive.stopMotors();
//
//                    myMechDrive.driveForward(SPD_DRIVE_MED, 3.2);
//                    break;
//
//                case RIGHT: // working
//                    myMechDrive.setMotorPowerStrafeRight(.5);
//                    linearOp.sleep(1500);
//
//                    myMechDrive.driveBackward(SPD_DRIVE_MED, 2);
//                    myGyro.gyroOrientMecanum(134, myMechDrive);
//                    myMechDrive.stopMotors();
//
//                    myMechDrive.driveForward(SPD_DRIVE_MED, 3.4);
//                    break;
//        }


    }
}

