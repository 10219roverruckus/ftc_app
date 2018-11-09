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
    public MecanumMineralMiner() {
    }

    // *********  Method used by both crater and depot to choose a mineral if camera does not work ******* //

    public void findingMineral() {          // do not delete backup in case camera does not work
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


    // ********* Method used by Crater and Depot to select mineral using Camera ************* //

    public void findingMineralCamera(double cameraGoldLocation) {

        // find location of the mineral using camera

        if (cameraGoldLocation < 300 && cameraGoldLocation > 1) {
            goldPosition = GoldPosition.MIDDLE;
        } else if (cameraGoldLocation > 300) {
            goldPosition = GoldPosition.RIGHT;
        } else {
            goldPosition = GoldPosition.LEFT;
        }
    }

    //************  Method used by both crater and depot to push off mineral   ***********  //

    public void driveMineral(GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor) {

        linearOp.telemetry.addData("MINERAL", goldPosition);
        linearOp.telemetry.update();
        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor

        myMechDrive.strafeRight(.3, .3);                    // get away from the lander
        myMechDrive.driveForward(.3, .3);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER


        switch (goldPosition) {                                            //Gyro angles robot to push off mineral
            case LEFT:
                myGyro.gyroOrientMecanum(36, myMechDrive);           // different (34)
                myMechDrive.stopMotors();
                myMechDrive.driveForward(SPD_DRIVE_MED, 2.15);    // Moves forward to push off mineral (Originally 1.8)
                break;

            case MIDDLE:
                myGyro.gyroOrientMecanum(4, myMechDrive);            //turning too much towards the right. Need to adjust?
                myMechDrive.stopMotors();
                myMechDrive.driveForward(SPD_DRIVE_MED, 1.7);      // Moves forward to push off mineral
                break;

            case RIGHT:
                myGyro.gyroOrientMecanum(-14, myMechDrive);          // Gyro angles appears correct.
                myMechDrive.stopMotors();
                myMechDrive.driveForward(SPD_DRIVE_MED, 2);        // Moves forward to push off mineral (Originally 1.8)
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
        while (hsvValues[0] > RED_THRESHOLD && hsvValues[0] < BLUE_THRESHOLD) {
            Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
                    (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
                    hsvValues);
            myMechDrive.setMotorSpeeds(-SPD_DRIVE_MED);
            //myMechDrive.setMotorSpeeds(SPD_DRIVE_MED);
            linearOp.idle();
        }
        myMechDrive.stopMotors();                                       //DRIVE FUNCTION DOESN'T HAVE A STOP.MOTORS IN IT

        myGyro.gyroOrientMecanum(74, myMechDrive);                //orients self with red tape so parallel to tape.
        myMechDrive.stopMotors();

        switch (goldPosition) {                                          //drive toward wall distance is different based on distance
            case LEFT:
                myMechDrive.driveForward(SPD_DRIVE_MED, 3.6);   // different distance to wall after backup to tape DO NOT CHANGE
                break;
            case MIDDLE:
                myMechDrive.driveForward(SPD_DRIVE_MED, 3.7);  // different distance to wall after backup to tape DO NOT CHANGE
                break;
            case RIGHT:
                myMechDrive.driveForward(SPD_DRIVE_MED, 3.9);  // different distance to wall after backup to tape used to be 4.2 but was too long DO NOT CHANGE
                break;
        }
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
                myMechDrive.driveForward(SPD_DRIVE_LOW, 1);
                myMechDrive.driveBackward(SPD_DRIVE_LOW, .2);

                myGyro.gyroOrientMecanum(-28, myMechDrive);       //rotates in toward depot
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 2.2);

                myGyro.gyroOrientMecanum(42, myMechDrive);        // 42 places team marker close to tape
                myMechDrive.stopMotors();                               // Recommend to adjust with adding degrees to turn

                myMechDrive.strafeLeft(.2, .3);           // Straffes so that team marker does not hit glass

                myTeamMarker.teamMarkerArmOutside();                   // drop team marker
                linearOp.sleep(1000);
                myTeamMarker.teamMarkerArmRaised();                     // raise arm

                myGyro.gyroOrientMecanum(130, myMechDrive);       //Orient gyro to move from depot to crater
                myMechDrive.stopMotors();
                break;

            case MIDDLE:                                                //pushes middle mineral into the depot to score
                myGyro.gyroOrientMecanum(0, myMechDrive);         // drive forward towards depot just a little bit
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 2.1);    // push mineral into depotfor extra point
                myMechDrive.driveBackward(SPD_DRIVE_LOW, .2);   // back up to clear mineral when turning

                myGyro.gyroOrientMecanum(60, myMechDrive);        // turn to drop team marker
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, .3);    // inch forward a little bit to ensure team marker drops in depot

                myTeamMarker.teamMarkerArmOutside();                   // drop team maker
                linearOp.sleep(1000);
                myTeamMarker.teamMarkerArmRaised();                     // raise arm

                myGyro.gyroOrientMecanum(130, myMechDrive);       // Orient gyro to move from depot to crater
                myMechDrive.stopMotors();
                break;

            case RIGHT:                                                   // pushes mineral into wall
                myMechDrive.driveForward(SPD_DRIVE_LOW, 1.1);     // Forward to push mineral to clear middle mineral
                myMechDrive.driveBackward(SPD_DRIVE_LOW, .2);     // backup so turn does not hit mineral
                myGyro.gyroOrientMecanum(32, myMechDrive);         //  rotates in toward depot
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 2.1);     // driving forward into depot

                myGyro.gyroOrientMecanum(60, myMechDrive);         // turn to drop team marker in depot
                myMechDrive.stopMotors();

                myMechDrive.strafeLeft(.2, .3);            // straffe so team marker does not hit glass

                myTeamMarker.teamMarkerArmOutside();                     // drop team marker
                linearOp.sleep(1000);
                myTeamMarker.teamMarkerArmRaised();                       // raise arm

                myGyro.gyroOrientMecanum(130, myMechDrive);        // Orient gyro to move from depot to crater
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
                myGyro.gyroOrientMecanum(130, myMechDrive);       // gyro corrects angle around plexiglass seam
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3);     // Shortest distance.... drives the rest of the way
                break;

            case MIDDLE:
                myMechDrive.setMotorPowerStrafeRight(.5);                //straffe to align with wall
                linearOp.sleep(2000);

                myMechDrive.driveForward(SPD_DRIVE_MED, 2);     //drive half way toward crater
                myGyro.gyroOrientMecanum(130, myMechDrive);       // gyro corrects angle around plexiglass seam
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3.2);   // Medium distance ... drives the rest of the way
                break;

            case RIGHT:
                myMechDrive.setMotorPowerStrafeRight(.5);                //straffe to align with wall
                linearOp.sleep(1500);

                myMechDrive.driveForward(SPD_DRIVE_MED, 2);     //drive half way toward crater
                myGyro.gyroOrientMecanum(130, myMechDrive);       // gyro corrects angle around plexiglass seam
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3.4);   // Longest distance... drives the rest of the way
                break;
        }

    }
    // NEW METHODS FOR PLEXIGLASS SOLUTION


    // ****** methods used for crater from going to crater from depot using a triangle gyro ******* //
    // testing some gyro triangles on the way back to the crater so the robot does not get caught on the seam

    public void wallToDepotGyro(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance, TeamMarker myTeamMarker) {
        myGyro.gyroOrientMecanum(137, myMechDrive);              // Orient for straight drive to depot
        myMechDrive.stopMotors();                                      // Stop motors

        myMechDrive.setMotorPowerStrafeRight(.3);                      // Align to wall
        linearOp.sleep(500);                                // new time = near plexiglass
        // linearOp.sleep(1500);                                        // Orignial time for strafing into plexiglass
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

        myGyro.gyroOrientMecanum(175, myMechDrive);         //rotate to drop team marker into depot was 170 but it needed to be a bigger angle
        myMechDrive.stopMotors();                                 // stop motors

        myMechDrive.strafeLeft(.2, .3);            // strafe away so the marker does not get stuck on wall


        myTeamMarker.teamMarkerArmOutside();                      // drop team maker
        linearOp.sleep(1250);
        myTeamMarker.teamMarkerArmRaised();
        linearOp.sleep(500);

        myMechDrive.strafeLeft(SPD_DRIVE_LOW, .2);         // get away from team maker to it does not get caught on the wheel
        myMechDrive.driveBackward(SPD_DRIVE_LOW, .7);       // was .7 but the tail of the robot wa hitting the wall

        //myGyro.gyroOrientMecanum(137, myMechDrive);               // Orignial angle Orient straight to park in crater... Angle between 136 - 139
        myGyro.gyroOrientMecanum(135.5, myMechDrive);         // new angle to make the triangle around the seam it was 133.1 but it angled too far out
        myMechDrive.stopMotors();                                   // 138 degrees forces us into the plexiglass
        //linearOp.sleep(500);                                      // Commenting out 11/8 5:45PM  to see if this is causing a 360 spin
        //linearOp.idle();                                          // Commenting out 11/8 5:45PM to see if this is causing a 360 spin

        myMechDrive.setMotorPowerStrafeRight(.2);                 // staffing into wall
        linearOp.sleep(1000);


        myMechDrive.driveBackward(SPD_DRIVE_MED, 2.0);    // Drive to park in crater

        // myGyro.gyroOrientMecanum(137, myMechDrive);               // Gyro correction for plexiglass. Same angle as above.

        myGyro.gyroOrientMecanum(138.4, myMechDrive);        // new angle to angle back to crater - finishing the triangle
        myMechDrive.stopMotors();
        linearOp.sleep(500);

        myMechDrive.driveBackward(SPD_DRIVE_MED, 3.3);    //Drive past plexiglass seam


    }
//**********      method used by Depot for driving from Depot to Crater   *****************//

    public void depotToCraterGyro(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {

        switch (goldPosition) {

            case LEFT:

                myMechDrive.setMotorPowerStrafeRight(.3);                   //straffe to align with wall
                linearOp.sleep(550);                            // was 1500 but I wanted to make it close to the wall and not hit the wall

                myMechDrive.driveForward(SPD_DRIVE_MED,.5);         // added another drive forward to delay the triangle to later

                myGyro.gyroOrientMecanum(138.2, myMechDrive);         // align robot for triangle around the plexiglass seam
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 1.5);       // drive first leg of the triangle

                myGyro.gyroOrientMecanum(132.6, myMechDrive);         // re-orient robot for second triangle leg toward depot
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3.5);       //drive second leg of triangle

                break;

            case MIDDLE:
                myMechDrive.setMotorPowerStrafeRight(.3);                   //straffe to align with wall
                linearOp.sleep(550);

                myMechDrive.driveForward(SPD_DRIVE_MED,.5);         // added another drive forward to delay the triangle to later

                myGyro.gyroOrientMecanum(138.2, myMechDrive);         // rotate away from wall to making triangle was 141.1 but too big of an angle
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 1.5);         // drive first leg of the triangle

                myGyro.gyroOrientMecanum(132.6, myMechDrive);           // re-orient robot for second triangle leg toward depot
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3.5);        //drive second leg of triangle

                break;
//
            case RIGHT:
                myMechDrive.setMotorPowerStrafeRight(.3);                     //straffe to align with wall
                linearOp.sleep(550);

                myMechDrive.driveForward(SPD_DRIVE_MED,.5);          // added another drive forward to delay the triangle to later

                myGyro.gyroOrientMecanum(138.2, myMechDrive);           //  was 141.1 but too big of an angle rotate away from wall to making triangle
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 1.5);        // drive first leg of the triangle

                myGyro.gyroOrientMecanum(132.6, myMechDrive);          // re-orient robot for second triangle leg toward depot
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3.5);       //drive second leg of triangle

                break;

        }
    }
}

