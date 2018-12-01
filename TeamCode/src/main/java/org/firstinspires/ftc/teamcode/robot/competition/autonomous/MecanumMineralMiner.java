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
    public final int RED_THRESHOLD = 30;                //maybe a higher threshold (12.5)
    public final int BLUE_THRESHOLD = 100;              //maybe a higher threshold (12.5)
    public final int COLOR_HUE_THRESHOLD = 125;
    float hsvValues[] = {0F, 0F, 0F};

    // created constant variables that are used for speed (different setting)

    final double SPD_DRIVE_LOW = .41;                  //Lowest speed
    final double SPD_DRIVE_MED = .38;                   //Default is  SPD_MED
    final double SPD_COLOR_REDUCE = 0.02;
    final double SPD_DRIVE_HIGH = .75;
    final double SPD_DRIVE_MAX = 1.0;
    final double SPD_ARM_MED = .5;
    final long sleepTime = 50;

    // variables and constants used by color sensor

    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }

    //in future, will pass Camera as Parameter
    public MecanumMineralMiner() {
    }

    // *********  Method used by both crater and depot to choose a mineral if camera does not work ******* //

    public void findingMineral() {          // do not delete backup in case camera does not work
//        if (cameraGoldLocation < 280) {
//            goldPosition = GoldPosition.LEFT;
//        }
//        else if (cameraGoldLocation > 500) {
//            goldPosition = GoldPosition.RIGHT;
//        else {
//            goldPosition = GoldPosition.MIDDLE;
//        }
        goldPosition = goldPosition.MIDDLE;
    }


    // ********* Method used by Crater and Depot to select mineral using Camera ************* //

    public void findingMineralCamera(double cameraGoldLocation) {

        // find location of the mineral using camera
//
//        if (cameraGoldLocation < 300 && cameraGoldLocation > 1) {
//            goldPosition = GoldPosition.MIDDLE;                           commented out while camera does not work
//        } else if (cameraGoldLocation > 300) {                            program works perfectly DO NOT CHANGE THE CODE
//            goldPosition = GoldPosition.RIGHT;
//        } else {
//            goldPosition = GoldPosition.LEFT;
//        }

        goldPosition = goldPosition.RIGHT;
    }

    //************  Method used by both crater and depot to push off mineral   ***********  //

    public void driveMineral(GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor) {

        linearOp.telemetry.addData("MINERAL", goldPosition);
        linearOp.telemetry.update();
        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor

        myMechDrive.strafeRight(SPD_DRIVE_LOW, .3);                    // get away from the lander
        linearOp.sleep(sleepTime);
        myMechDrive.driveForward(SPD_DRIVE_MED, .3);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);

        switch (goldPosition) {                                            //Gyro angles robot to push off mineral
            case LEFT:
                myGyro.gyroOrientMecanum(39, myMechDrive);           // different (34)
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myMechDrive.driveForward(SPD_DRIVE_MED, 2.15);    // Moves forward to push off mineral (Originally 1.8)
                linearOp.sleep(sleepTime);
                break;

            case MIDDLE:
                myGyro.gyroOrientMecanum(4, myMechDrive);            //turning too much towards the right. Need to adjust?
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myMechDrive.driveForward(SPD_DRIVE_MED, 1.7);      // Moves forward to push off mineral
                linearOp.sleep(sleepTime);
                break;

            case RIGHT:
                myGyro.gyroOrientMecanum(-32, myMechDrive);          // Gyro angles appears correct.
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myMechDrive.driveForward(SPD_DRIVE_MED, 1.7);        // Moves forward to push off mineral (Originally 1.8)
                linearOp.sleep(sleepTime);
                break;
        }
    }

    // ******   Method used by crater to go from Mineral to Wall  (Drives back, stops at tape, and orients towards wall)  *****  //

    public void craterMineralToWall(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {

        myMechDrive.stopMotors();

        Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),     // Move backwards until color detected
                (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
                (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
//        while (hsvValues[0] > RED_THRESHOLD && hsvValues[0] < BLUE_THRESHOLD) {
//            Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
//                    (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
//                    (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
//                    hsvValues);
//            myMechDrive.setMotorSpeeds(-SPD_DRIVE_MED);
//            //myMechDrive.setMotorSpeeds(SPD_DRIVE_MED);
//            linearOp.idle();
//        }

        while (hsvValues[0] < COLOR_HUE_THRESHOLD) {
            Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            myMechDrive.setMotorSpeeds(-SPD_DRIVE_MED + SPD_COLOR_REDUCE);
            //myMechDrive.setMotorSpeeds(SPD_DRIVE_MED);
            linearOp.idle();
        }

        myMechDrive.stopMotors();                                       //DRIVE FUNCTION DOESN'T HAVE A STOP.MOTORS IN IT
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(74, myMechDrive);                //orients self with red tape so parallel to tape.
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);
        switch (goldPosition) {                                          //drive toward wall distance is different based on distance
            case LEFT:
                myMechDrive.driveForward(SPD_DRIVE_MED, 3.4);   // different distance to wall after backup to tape DO NOT CHANGE
                break;
            case MIDDLE:
                myMechDrive.driveForward(SPD_DRIVE_MED, 3.55);  // was 3.8 but it was too big of a distance different distance to wall after backup to tape DO NOT CHANGE
                break;
            case RIGHT:
                myMechDrive.driveForward(SPD_DRIVE_MED, 3.8);  // different distance to wall after backup to tape used to be 4.2 but was too long DO NOT CHANGE
                break;
        }
        linearOp.sleep(sleepTime);
    }

    // *****   Method used for Crater to drive from along wall to Depot  ********//

    public void wallToDepot(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance, TeamMarker myTeamMarker) {

        myGyro.gyroOrientMecanum(137, myMechDrive);              // Orient for straight drive to depot
        myMechDrive.stopMotors();                                      // Stop motors

        myMechDrive.setMotorPowerStrafeRight(.3);                      // Align to wall
        linearOp.sleep(1500);                               // Time for straffing
        myMechDrive.stopMotors();                                      // Stop motors

        myMechDrive.driveForward(SPD_DRIVE_MED, 3);           //going toward depot using color sensor

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

        myMechDrive.stopMotors();                                 // Robot is now in Depot

        myGyro.gyroOrientMecanum(170, myMechDrive);         //rotate to drop team marker into depot
        myMechDrive.stopMotors();                                 // stop motors

        myMechDrive.strafeLeft(.2, .3);            // strafe away so the marker does not get stuck on wall

        myTeamMarker.teamMarkerArmOutside();                      // drop team maker
        linearOp.sleep(1250);
        myTeamMarker.teamMarkerArmRaised();
        linearOp.sleep(500);

        myMechDrive.strafeLeft(SPD_DRIVE_LOW, .2);         // get away from team maker to it does not get caught on the wheel
        myMechDrive.driveBackward(SPD_DRIVE_LOW, .7);


        myMechDrive.stopMotors();

        // was what Mr. Duval had
        myGyro.gyroOrientMecanum(137, myMechDrive);         // Orient straight to park in crater... Angle between 136 - 139

        myMechDrive.stopMotors();                                 // 138 degrees forces us into the plexiglass
        linearOp.sleep(500);
        linearOp.idle();

        myMechDrive.setMotorPowerStrafeRight(.3);                 // staffing into wall
        linearOp.sleep(1000);
        //Emma did this
        myGyro.gyroOrientMecanum(133.5, myMechDrive);
        myMechDrive.driveBackward(SPD_DRIVE_MED, 2.0);    // Drive to park in crater

        myGyro.gyroOrientMecanum(135.5, myMechDrive);         // Gyro correction for plexiglass. Same angle as above.
        myMechDrive.stopMotors();
        linearOp.sleep(500);

        myMechDrive.driveBackward(SPD_DRIVE_MED, 3.3);    //Drive past plexiglass seam

    }


    // ******* methods used by crater for double sampling *********


    // *******  method used for Crater to drive to depot to drop off team marker *********
    public void firstMineralToTeamMarker(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance, TeamMarker myTeamMarker) {

        myGyro.gyroOrientMecanum(137, myMechDrive);              // Orient for straight drive to depot
        myMechDrive.stopMotors();                                      // Stop motors

        myMechDrive.setMotorPowerStrafeRight(.3);                      // Align to wall
        linearOp.sleep(1500);                               // Time for straffing
        myMechDrive.stopMotors();                                      // Stop motors

        myMechDrive.driveForward(SPD_DRIVE_MED, 3);           //going toward depot using color sensor

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

        myMechDrive.stopMotors();                                 // Robot is now in Depot

        myGyro.gyroOrientMecanum(170, myMechDrive);         //rotate to drop team marker into depot
        myMechDrive.stopMotors();                                 // stop motors

        myMechDrive.strafeLeft(.2, .3);            // strafe away so the marker does not get stuck on wall


        myTeamMarker.teamMarkerArmOutside();                      // drop team maker
        linearOp.sleep(1250);
        myTeamMarker.teamMarkerArmRaised();
        linearOp.sleep(500);

    }

    // ***** method used for crater to double sampling *******

    public void knockOffSecondMineral (GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {

        double angleHolder = 0;                                       // fake angle until real one is found

        myMechDrive.strafeLeft(SPD_DRIVE_LOW, .2);           // get away from team maker to it does not get caught on the wheel


        myGyro.gyroOrientMecanum(92, myMechDrive );          // angle that will be facing towards minerals

        switch (goldPosition) {
            case LEFT:
                myGyro.gyroOrientMecanum(61, myMechDrive);      // gyro towards mineral
                myMechDrive.stopMotors();
                myMechDrive.driveBackward(SPD_DRIVE_MED, .7);   // drive backwards to drop of mineral
                linearOp.sleep(1000);
                myMechDrive.driveForward(SPD_DRIVE_MED, .7);    // drive forward to go back into the depot
                linearOp.sleep(1000);
                myGyro.gyroOrientMecanum(137, myMechDrive);      // angle robot parallel to the  wall
                myMechDrive.stopMotors();

            case MIDDLE:

                myGyro.gyroOrientMecanum(92, myMechDrive);      // gyro towards mineral
                myMechDrive.stopMotors();
                myMechDrive.driveBackward(SPD_DRIVE_MED, .5);
                linearOp.sleep(1000);// drive backwards to drop of mineral
                myMechDrive.driveForward(SPD_DRIVE_MED, .5);    // drive forward to go back into the depot
                linearOp.sleep(1000);
                myGyro.gyroOrientMecanum(137, myMechDrive);      // angle robot parallel to the  wall
                myMechDrive.stopMotors();

            case RIGHT:

                myGyro.gyroOrientMecanum(137, myMechDrive);      // gyro towards mineral
                myMechDrive.stopMotors();
                myMechDrive.driveBackward(SPD_DRIVE_MED, .7);   // drive backwards to drop of mineral
                linearOp.sleep(1000);
                myMechDrive.driveForward(SPD_DRIVE_MED, .7);    // drive forward to go back into the depot
                linearOp.sleep(1000);
                myGyro.gyroOrientMecanum(137, myMechDrive);      // angle robot parallel to the  wall
                myMechDrive.stopMotors();
        }
    }

    // ***** method used my crater to drive back to park in crater ********

    public void driveBackToCrater (GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {

        myGyro.gyroOrientMecanum(137, myMechDrive);         // Orient straight to park in crater... Angle between 136 - 139
        myMechDrive.stopMotors();                                 // 138 degrees forces us into the plexiglass
        linearOp.sleep(500);
        linearOp.idle();

        myMechDrive.setMotorPowerStrafeRight(.3);                 // staffing into wall
        linearOp.sleep(1000);

        myMechDrive.driveBackward(SPD_DRIVE_MED, 2.0);    // Drive to park in crater

        myGyro.gyroOrientMecanum(137, myMechDrive);         // Gyro correction for plexiglass. Same angle as above.
        myMechDrive.stopMotors();
        linearOp.sleep(500);

        myMechDrive.driveBackward(SPD_DRIVE_MED, 3.3);    //Drive past plexiglass seam


    }

















    // *****   Method used by Depot to move from Mineral to Depot   *******  //

    public void mineralToDepot(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance, TeamMarker myTeamMarker) {
        switch (goldPosition) {

            case LEFT:                                                  // pushes mineral towards wall.  This case will cause mineral to move from depot to crater
//                myMechDrive.driveForward(SPD_DRIVE_LOW, 1);
//                myMechDrive.driveBackward(SPD_DRIVE_LOW, .2);
//
//                myGyro.gyroOrientMecanum(31, myMechDrive);       //rotates in toward depot
//                myMechDrive.stopMotors();
//
//                myMechDrive.driveForward(SPD_DRIVE_MED, 2.2);

                myMechDrive.driveForward(SPD_DRIVE_MED, .9);
                myGyro.gyroOrientMecanum(44, myMechDrive);
                myMechDrive.stopMotors();

                myMechDrive.strafeRight(SPD_DRIVE_MED, 2.3);
//
//                myGyro.gyroOrientMecanum(88, myMechDrive);        // it was 42  which places the team marker close to tape
//                myMechDrive.stopMotors();                               // Recommend to adjust with adding degrees to turn
//
//                myMechDrive.strafeLeft(.2, .4);           // Straffes so that team marker does not hit glass

                myTeamMarker.teamMarkerArmOutside();                   // drop team marker
                linearOp.sleep(1000);
                myTeamMarker.teamMarkerArmRaised();                     // raise arm
                myMechDrive.driveBackward(SPD_DRIVE_MED, .1);
                linearOp.sleep(sleepTime);
                myGyro.gyroOrientMecanum(133, myMechDrive);       //Orient gyro to move from depot to crater
                linearOp.sleep(sleepTime);
                break;

            case MIDDLE:                                                //pushes middle mineral into the depot to score
                myGyro.gyroOrientMecanum(0, myMechDrive);         // drive forward towards depot just a little bit
//                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 2.4);    // push mineral into depot for extra point

//                myMechDrive.driveBackward(SPD_DRIVE_LOW, .2);   // back up to clear mineral when turning

                myGyro.gyroOrientMecanum(90, myMechDrive);        // turn to drop team marker was 60 but it was to small of an angle
                myMechDrive.stopMotors();

                myTeamMarker.teamMarkerArmOutside();                   // drop team maker
                linearOp.sleep(1000);
                myTeamMarker.teamMarkerArmRaised();                     // raise arm


                myMechDrive.driveForward(SPD_DRIVE_MED, .4);
                myGyro.gyroOrientMecanum(133, myMechDrive);       // Orient gyro to move from depot to crater
                myMechDrive.stopMotors();

                break;

            case RIGHT:                                                   // pushes mineral into wall
                myMechDrive.driveForward(SPD_DRIVE_MED, 1.1);     // Forward to push mineral to clear middle mineral
                myMechDrive.driveBackward(SPD_DRIVE_MED, .2);     // backup so turn does not hit mineral
                myGyro.gyroOrientMecanum(35, myMechDrive);         //  was 32 but it did not ratate enough rotates in toward depot
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3);     // was 2.1 but it did not go far enough driving forward into depot

                myGyro.gyroOrientMecanum(90, myMechDrive);         // was 88 but to small of an angle turn to drop team marker in depot was 60 but it was to small of an angle
                myMechDrive.stopMotors();

                myMechDrive.strafeLeft(SPD_DRIVE_LOW, .3);            // straffe so team marker does not hit glass

                myTeamMarker.teamMarkerArmOutside();                     // drop team marker
                linearOp.sleep(1000);
                myTeamMarker.teamMarkerArmRaised();                       // raise arm
                linearOp.sleep(500);
                myGyro.gyroOrientMecanum(133, myMechDrive); // Orient gyro to move from depot to crater
                myMechDrive.driveForward(SPD_DRIVE_MED, .3);
                myMechDrive.strafeRight(SPD_DRIVE_LOW, .2);
                myMechDrive.stopMotors();
                break;
        }

        myMechDrive.stopMotors();

    }

//  ***********   Method used for Depot to drive from depot to crater   ******************* //

    public void depotToCrater(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {


        switch (goldPosition) {

            case LEFT:

                myMechDrive.setMotorPowerStrafeRight(.5);                //straffe to align with wall
                linearOp.sleep(1500);

                myMechDrive.driveForward(SPD_DRIVE_MED, 2);     //drive half way toward crater
                myGyro.gyroOrientMecanum(133, myMechDrive);       // gyro corrects angle around plexiglass seam
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3);     // Shortest distance.... drives the rest of the way
                break;

            case MIDDLE:
                myMechDrive.setMotorPowerStrafeRight(.5);                //straffe to align with wall
                linearOp.sleep(2000);

                myMechDrive.driveForward(SPD_DRIVE_MED, 2);     //drive half way toward crater
                myGyro.gyroOrientMecanum(133, myMechDrive);       // gyro corrects angle around plexiglass seam
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3.2);   // Medium distance ... drives the rest of the way
                break;

            case RIGHT:
                myMechDrive.setMotorPowerStrafeRight(.5);                //straffe to align with wall
                linearOp.sleep(1500);

                myMechDrive.driveForward(SPD_DRIVE_MED, 2);     //drive half way toward crater
                myGyro.gyroOrientMecanum(133, myMechDrive);       // gyro corrects angle around plexiglass seam
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3.4);   // Longest distance... drives the rest of the way
                break;
        }

    }
    // NEW METHODS FOR PLEXIGLASS SOLUTION


    // ****** methods used for crater from going to crater from depot using a triangle gyro ******* //
    // testing some gyro triangles on the way back to the crater so the robot does not get caught on the seam

    public void wallToDepotGyro(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance, TeamMarker myTeamMarker) {
        //129 - change to 130 to drive away from wall a litte
        myGyro.gyroOrientMecanum(130, myMechDrive);              // Orient for straight drive to depot
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);
//        myMechDrive.setMotorPowerStrafeRight(.3);                      // Align to wall
//        linearOp.sleep(500);                                // new time = near plexiglass
//        myMechDrive.stopMotors();                                      // Stop motors
        myMechDrive.strafeRight(SPD_DRIVE_LOW, .36);
        // linearOp.sleep(1500);                                        // Orignial time for strafing into plexiglass
        linearOp.sleep(sleepTime);
//        myMechDrive.driveForward(SPD_DRIVE_MED, 3);           //going toward depot using color sensor

        Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
                (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
                (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        while (hsvValues[0] < COLOR_HUE_THRESHOLD) {
            Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            myMechDrive.setMotorSpeeds(SPD_DRIVE_MED - SPD_COLOR_REDUCE);
            linearOp.idle();
        }

        myMechDrive.stopMotors();                                 // Robot is now in Depot
        linearOp.sleep(sleepTime);
        myMechDrive.driveBackward(SPD_DRIVE_MED, .05);
        linearOp.sleep(sleepTime);
        myMechDrive.strafeLeft(.3, .3);            // strafe away so the marker does not get stuck on wall
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(175, myMechDrive);         //rotate to drop team marker into depot was 170 but it needed to be a bigger angle
        myMechDrive.stopMotors();                                 // stop motors
        linearOp.sleep(sleepTime);

        myTeamMarker.teamMarkerArmOutside();
        // drop team maker
        //1250
        linearOp.sleep(500);
        myTeamMarker.teamMarkerArmRaised();
        //500
        linearOp.sleep(250);






        myMechDrive.strafeLeft(SPD_DRIVE_LOW, .35);         // get away from team maker to it does not get caught on the wheel
        linearOp.sleep(sleepTime);
        myMechDrive.driveBackward(SPD_DRIVE_LOW, .18);       // was .7 but the tail of the robot wa hitting the wall
        linearOp.sleep(sleepTime);

        myGyro.gyroReset();
        //-48
        myGyro.gyroOrientMecanum(-47, myMechDrive);
        myGyro.gyroOrientMecanum(-47, myMechDrive);
        linearOp.sleep(sleepTime);
        myMechDrive.driveBackward(SPD_DRIVE_MED, 3);    // Drive to park in crater
        linearOp.sleep(sleepTime);
//-43
        myGyro.gyroOrientMecanum(-42, myMechDrive);
        myGyro.gyroOrientMecanum(-42, myMechDrive);
        linearOp.sleep(sleepTime);
        myMechDrive.driveBackward(SPD_DRIVE_MED, 2.3);    // Drive to park in crater
        linearOp.sleep(sleepTime);
        /*

        // testing for next line
//      myMechDrive.rotateRight(SPD_DRIVE_MED, .2);          // adding angle to make sure the strafing does not mess up the angle making it go over the line
//        linearOp.sleep(sleepTime);
        //myGyro.gyroOrientMecanum(137, myMechDrive);               // Orignial angle Orient straight to park in crater... Angle between 136 - 139
        myGyro.gyroOrientMecanum(129.5, myMechDrive);         // new angle to make the triangle around the seam it was 133.1 but it angled too far out
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(129.5, myMechDrive);         // new angle to make the triangle around the seam it was 133.1 but it angled too far out
//  myMechDrive.stopMotors();                                   // 138 degrees forces us into the plexiglass
        linearOp.sleep(sleepTime);
        //linearOp.sleep(500);                                      // Commenting out 11/8 5:45PM  to see if this is causing a 360 spin
        //linearOp.idle();                                          // Commenting out 11/8 5:45PM to see if this is causing a 360 spin

//        myMechDrive.setMotorPowerStrafeRight(.2);                 // staffing into wall
//        linearOp.sleep(1000);
        myMechDrive.strafeRight(SPD_DRIVE_LOW, .15);
//        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        // for testing 11/14/18
        //was 133
        //was 131
        //was 126
//        myGyro.gyroOrientMecanum(127, myMechDrive);         //135 - greater numbers = more towards wall - points towards wall too much was correcting angle so the robot does not go into the wall was 132.5 but the angle
//        linearOp.sleep(sleepTime);

// GO BACK TO CRATER!!!!
        myMechDrive.driveBackward(SPD_DRIVE_MED, 3);    // Drive to park in crater
        linearOp.sleep(sleepTime);

        //132
        myGyro.gyroOrientMecanum(130, myMechDrive);         //135 was correcting angle so the robot does not go into the wall was 132.5 but the angle
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(130, myMechDrive);         //135 was correcting angle so the robot does not go into the wall was 132.5 but the angle
        linearOp.sleep(sleepTime);
        myMechDrive.driveBackward(SPD_DRIVE_MED, 2.4);    // Drive to park in crater
        linearOp.sleep(sleepTime);
        */
    }
//**********      method used by Depot for driving from Depot to Crater   *****************//

    public void depotToCraterGyro(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {

        switch (goldPosition) {

            case LEFT:

//                myMechDrive.setMotorPowerStrafeRight(.3);                   //straffe to align with wall
//                linearOp.sleep(1000);                            // was 1500 but I wanted to make it close to the wall and not hit the wall

//                myMechDrive.driveForward(SPD_DRIVE_MED,.5);         // added another drive forward to delay the triangle to later
//
////                myGyro.gyroOrientMecanum(138.2, myMechDrive);         // align robot for triangle around the plexiglass seam
////                myMechDrive.stopMotors();
//
//                myMechDrive.driveForward(SPD_DRIVE_MED, 1.5);       // drive first leg of the triangle
//
////                myGyro.gyroOrientMecanum(132.6, myMechDrive);         // re-orient robot for second triangle leg toward depot
////                myMechDrive.stopMotors();
//
//                myMechDrive.driveForward(SPD_DRIVE_MED, 3.8);       //drive second leg of triangle
                myMechDrive.driveForward(SPD_DRIVE_MED, 4);
                linearOp.sleep(sleepTime);
                myMechDrive.strafeRight(SPD_DRIVE_LOW, .265);
                linearOp.sleep(sleepTime);
                myMechDrive.driveForward(SPD_DRIVE_MED, 1.6);

                break;

            case MIDDLE:
//                myMechDrive.setMotorPowerStrafeRight(.3);                   //straffe to align with wall
//                linearOp.sleep(1000);

                myMechDrive.driveForward(SPD_DRIVE_MED,.3);         // added another drive forward to delay the triangle to later
                linearOp.sleep(sleepTime);
                myMechDrive.strafeRight(SPD_DRIVE_LOW, .24);
                linearOp.sleep(sleepTime);
                myMechDrive.driveForward(SPD_DRIVE_MED,4.5);
                linearOp.sleep(sleepTime);
                myMechDrive.strafeRight(SPD_DRIVE_LOW, .23);
                myMechDrive.driveForward(SPD_DRIVE_MED, .8);
//                myGyro.gyroOrientMecanum(138.2, myMechDrive);         // rotate away from wall to making triangle was 141.1 but too big of an angle
//                myMechDrive.stopMotors();

//                myMechDrive.driveForward(SPD_DRIVE_MED, 1.5);         // drive first leg of the triangle

//                myGyro.gyroOrientMecanum(132.6, myMechDrive);           // re-orient robot for second triangle leg toward depot
//                myMechDrive.stopMotors();

//                myMechDrive.driveForward(SPD_DRIVE_MED, 3.8);        //drive second leg of triangle

                break;
//
            case RIGHT:
                myMechDrive.setMotorPowerStrafeRight(.3);                     //straffe to align with wall
                linearOp.sleep(1000);

                myMechDrive.driveForward(SPD_DRIVE_MED,.5);          // added another drive forward to delay the triangle to later

                myGyro.gyroOrientMecanum(138.2, myMechDrive);           //  was 141.1 but too big of an angle rotate away from wall to making triangle
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 2);        // drive first leg of the triangle

                myGyro.gyroOrientMecanum(129, myMechDrive);          // re-orient robot for second triangle leg toward depot was 132.6 but it ran into the wall to much then it was 130 but it was still too big
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3);       //drive second leg of triangle

                break;

        }
    }
}

