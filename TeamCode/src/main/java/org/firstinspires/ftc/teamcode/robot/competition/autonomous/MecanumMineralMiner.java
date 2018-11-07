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

    public void findingMineralCamera (double cameraGoldLocation) {

        // find location of the mineral using camera

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


        //get away from the lander
        myMechDrive.strafeRight(.3,.3);
        myMechDrive.driveForward(.3, .3); //DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER




        // based on location go to certain case
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


        // goes backward towards tape near lander
        // detects red or blue


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


        //rotate towards wall
        myGyro.gyroOrientMecanum(74, myMechDrive);
        myMechDrive.stopMotors();  //orients self with red tape so parallel to tape.
        //different (72)


        //IN THEORY, THE DRIVE DISTANCE SHOULD BE THE SAME OR CLOSE FOR L / M / R
        //DRIVES TO WALL not


        //drive toward wall distance is different based on distance
        switch (goldPosition) {
            case LEFT:
                myMechDrive.driveForward(SPD_DRIVE_MED, 3.2);   // different distance to wall after backup to tape
                break;
            case MIDDLE:
                myMechDrive.driveForward(SPD_DRIVE_MED, 3.7);  // different distance to wall after backup to tape
                break;
            case RIGHT:
                myMechDrive.driveForward(SPD_DRIVE_MED, 4.2);  // different distance to wall after backup to tape
                break;
        }
    }

    //Method used for Crater to drive from along wall to Depot

    public void wallToDepot(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance, TeamMarker myTeamMarker) {

        myGyro.gyroOrientMecanum(137, myMechDrive);              // Orient for straight drive to depot
        myMechDrive.stopMotors();                                      // Stop motors
        myMechDrive.setMotorPowerStrafeRight(.3);                      // Align to wall
        linearOp.sleep(1500);                              // Time for staffing
        myMechDrive.stopMotors();                                      // Stop motors

        myMechDrive.driveForward(SPD_DRIVE_MED, 3);   //going toward depot using color sensor
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

        myMechDrive.stopMotors();                             // Robot is now in Depot

        myGyro.gyroOrientMecanum(170, myMechDrive);     //rotate to drop team marker in sepot
        myMechDrive.stopMotors();                             // stop motors

        myMechDrive.strafeLeft(.2, .3);       // strafe away so the marker does not get stuck on wall


        myTeamMarker.teamMarkerArmOutside();                  // drop team maker
        linearOp.sleep(1250);
        myTeamMarker.teamMarkerArmRaised();
        linearOp.sleep(500);

        myMechDrive.strafeLeft(SPD_DRIVE_LOW,.2);     // get away from team maker to it does not get caught on the wheel
        myMechDrive.driveBackward(SPD_DRIVE_LOW,.7);

        myGyro.gyroOrientMecanum(137, myMechDrive);     // Orient straight to park in crater... Angle between 136 - 139
        myMechDrive.stopMotors();                             // 138 forces us into the plexiglass
        linearOp.idle();

        myMechDrive.setMotorPowerStrafeRight(.3);              // staffe into wall
        linearOp.sleep(1000);

        myMechDrive.driveBackward(SPD_DRIVE_MED, 2.0); //Drive to park in crater

        myGyro.gyroOrientMecanum(137, myMechDrive);       // Gyro correction for plexiglass. Same angle as above.
        myMechDrive.stopMotors();

        myMechDrive.driveBackward(SPD_DRIVE_MED, 3.3);  //Drive past plexiglass seam

    }

// Method used by Depot to move from Mineral to Depot
    public void mineralToDepot (GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance, TeamMarker myTeamMarker) {
        switch (goldPosition) {

            case LEFT:
                myMechDrive.driveForward(SPD_DRIVE_LOW,1);
                myMechDrive.driveBackward(SPD_DRIVE_LOW,.2);

                myGyro.gyroOrientMecanum(-28, myMechDrive);        //rotates in toward depot
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED,2.2);

                myGyro.gyroOrientMecanum(42, myMechDrive);        // 42 places team marker close to tape
                myMechDrive.stopMotors();

                myMechDrive.strafeLeft(.2,.3);          // rotations .3 (perfect)

                myTeamMarker.teamMarkerArmOutside ();
                linearOp.sleep(1000);
                myTeamMarker.teamMarkerArmRaised();

                myGyro.gyroOrientMecanum(130, myMechDrive);
                myMechDrive.stopMotors();
                break;

            //push mineral into the depo
            //rotate and strafe to drop into the mineral

            case MIDDLE:
                myGyro.gyroOrientMecanum(0, myMechDrive);        //drive forward towards depot just a little bit
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED,2.1);  // push mineral into depotfor extra point
                myMechDrive.driveBackward(SPD_DRIVE_LOW, .2); // back up to clear mineral when turning

                myGyro.gyroOrientMecanum(60, myMechDrive);      // turn to drop team marker
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED,.3);   // inch forward a little bit to ensure team marker drops in depot

                myTeamMarker.teamMarkerArmOutside ();                 // drop team maker
                linearOp.sleep(1000);
                myTeamMarker.teamMarkerArmRaised();                   // lift servo arm

                myGyro.gyroOrientMecanum(130, myMechDrive);     // orient towards depot
                myMechDrive.stopMotors();
                break;

                // push mineral into the other wall and rotate away from depot and drive foward
                // rotate and strafe to drop of the team marker

            case RIGHT:
                myMechDrive.driveForward(SPD_DRIVE_LOW,1.1);     // Forward to push mineral to clear middle mineral
                myMechDrive.driveBackward(SPD_DRIVE_LOW,.2);     // backup so turn does not hit mineral
                myGyro.gyroOrientMecanum(32, myMechDrive);         //  rotates in toward depot
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED,2.1);     // driving forward for

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

        myMechDrive.stopMotors();




    }
//  Method used for Depot
    public void depotToCrater (GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {

        switch (goldPosition) {


            case LEFT:

                myMechDrive.setMotorPowerStrafeRight(.5);  //align with wall
                linearOp.sleep(1500);

                myMechDrive.driveForward(SPD_DRIVE_MED, 2); //drive half way toward crater
                myGyro.gyroOrientMecanum(130, myMechDrive); // gyro corrects angle
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3); // drives the rest of the way
                break;

            case MIDDLE:
                myMechDrive.setMotorPowerStrafeRight(.5);
                linearOp.sleep(2000);

                myMechDrive.driveForward(SPD_DRIVE_MED, 2); //drive half way toward crater
                myGyro.gyroOrientMecanum(130, myMechDrive); // gyro corrects angle
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3.2); // drives the rest of the way
                break;

            case RIGHT: // working
                myMechDrive.setMotorPowerStrafeRight(.5);
                linearOp.sleep(1500);

                myMechDrive.driveForward(SPD_DRIVE_MED, 2); //drive half way toward crater
                myGyro.gyroOrientMecanum(130, myMechDrive); // gyro corrects angle
                myMechDrive.stopMotors();

                myMechDrive.driveForward(SPD_DRIVE_MED, 3.4); // drives the rest of the way
                break;
        }


        // a method that I was going to do but found out the code is already in another method
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

