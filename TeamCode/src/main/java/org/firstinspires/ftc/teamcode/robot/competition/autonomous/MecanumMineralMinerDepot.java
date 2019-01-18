package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotator;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;


public class MecanumMineralMinerDepot {

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
    final long sleepTime = 100;
    final int servoRotateTeamMarker = 2000;

    // variables and constants used by color sensor

    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }

    //in future, will pass Camera as Parameter
    public MecanumMineralMinerDepot() {
    }


    // new Methods
    public void findingMineralCamera(double cameraGoldLocation) {

        //find location of the mineral using camera

        if (cameraGoldLocation < 200 && cameraGoldLocation > 1) {
            goldPosition = GoldPosition.RIGHT;                           //commented out while camera does not work
        } else if (cameraGoldLocation > 200) {                            //program works perfectly DO NOT CHANGE THE CODE
            goldPosition = GoldPosition.MIDDLE;
        } else {
            goldPosition = GoldPosition.LEFT;
        }
//        goldPosition = GoldPosition.MIDDLE;
    }



//    public void driveMineral(GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotator myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo) {
//        linearOp.telemetry.addData("MINERAL", goldPosition);
//        linearOp.telemetry.update();
////        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor
////        linearOp.sleep(sleepTime);
//        myMechDrive.strafeLeft(SPD_DRIVE_HIGH, .5);                    // get away from the lander
//        linearOp.sleep(sleepTime);
//        myMechDrive.driveForward(.3, .3);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
//        linearOp.sleep(sleepTime);
//        myMechDrive.strafeRight(SPD_DRIVE_HIGH, .51);
//        linearOp.sleep(sleepTime);
//        myGyro.gyroOrientMecanum(0, myMechDrive);           // different (34)
//        linearOp.sleep(sleepTime);
//        switch (goldPosition) {                                            //Gyro angles robot to push off mineral
//            case LEFT:
//                myMechDrive.driveForward(SPD_DRIVE_HIGH, .5);
//                linearOp.sleep(sleepTime);
//                myMechDrive.strafeLeft(SPD_DRIVE_HIGH, 2.9);
//                linearOp.sleep(sleepTime);
//                myMechDrive.rotateRight(SPD_DRIVE_MED, .5);         // fixing Gyro issue
//                linearOp.sleep(sleepTime);
//
//                myGyro.gyroOrientMecanum(-30, myMechDrive);           // different (34)
//                myMechDrive.stopMotors();
//                linearOp.sleep(sleepTime);
//                myMechDrive.driveBackward(SPD_DRIVE_HIGH, .4);
//                linearOp.sleep(sleepTime);
//
//                myIntakeRotator.mineralRotateLowerEncoder();
//                linearOp.sleep(sleepTime);
//
//                myIntakeExtenderArm.extendIntakeArmAllTheWay();
//                linearOp.sleep(sleepTime);
//
//                myIntakeServo.IntakeServoReverseTime();
//                linearOp.sleep(sleepTime);
//
//                myIntakeExtenderArm.retractIntakeArmAuto();
//                linearOp.sleep(sleepTime);
//
//                myIntakeRotator.mineralRotateRaiseEncoder();
//                linearOp.sleep(sleepTime);
//                break;
//
//            case MIDDLE:
//                //myMechDrive.strafeLeft(SPD_DRIVE_HIGH, .3);
//                //linearOp.sleep(sleepTime);
//
//                myGyro.gyroOrientMecanum(0, myMechDrive);            //turning too much towards the right. Need to adjust?
//                myMechDrive.stopMotors();
//                linearOp.sleep(sleepTime);
//
//                myIntakeRotator.mineralRotateLowerEncoder();
//                linearOp.sleep(sleepTime);
//                myIntakeExtenderArm.extendIntakeArmAllTheWay();
//                linearOp.sleep(sleepTime);
//
//
//                myIntakeServo.IntakeServoReverseTime();
//                linearOp.sleep(sleepTime);
//
//
//                myIntakeExtenderArm.retractIntakeArmAuto();
//                linearOp.sleep(sleepTime);
//                myIntakeExtenderArm.retractPowerAuto(1);
//                linearOp.sleep(1000);
//                myIntakeExtenderArm.stopIntakeArm();
//                myIntakeRotator.mineralRotateRaiseEncoder();
//                linearOp.sleep(sleepTime);
//                break;
//
//            case RIGHT:
//
//                myMechDrive.driveForward(SPD_DRIVE_HIGH, .5);
//                linearOp.sleep(sleepTime);
//                myMechDrive.strafeRight(SPD_DRIVE_HIGH, 2.8);
//                linearOp.sleep(sleepTime);
//                myMechDrive.rotateLeft(SPD_DRIVE_MED, .5);         // fixing Gyro issue
//                linearOp.sleep(sleepTime);
//
//                myGyro.gyroOrientMecanum(28, myMechDrive);           // different (34)
//                myMechDrive.stopMotors();
//                linearOp.sleep(sleepTime);
//                myMechDrive.driveBackward(SPD_DRIVE_HIGH, .2);
//                linearOp.sleep(sleepTime);
//
//                myMechDrive.strafeLeft(SPD_DRIVE_MED, .2);
//
//
//                myIntakeRotator.mineralRotateLowerEncoder();
//                linearOp.sleep(sleepTime);
//
//                myMechDrive.driveForward(SPD_DRIVE_MED, .8);
//
//                myIntakeExtenderArm.extendIntakeArmAllTheWay();
//                linearOp.sleep(sleepTime);
//
//                myIntakeServo.IntakeServoReverseTime();
//                linearOp.sleep(200);
//
//                myIntakeExtenderArm.retractIntakeArmAuto();
//                linearOp.sleep(sleepTime);
//
//                myIntakeRotator.mineralRotateRaiseEncoder();
//                linearOp.sleep(sleepTime);
//
//                myMechDrive.driveForward(SPD_DRIVE_HIGH, .4);
//                break;
//        }
//    }




    public void driveMineral (GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotator myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo) {
        linearOp.telemetry.addData("MINERAL", goldPosition);
        linearOp.telemetry.update();
//        linearOp.sleep(3500);  //SIMULATE LOWING ROBOT - COMMENT OUT WHEN RUNNING LIFT MOTOR!
        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor
       linearOp.sleep(sleepTime);
        myMechDrive.strafeLeft(SPD_DRIVE_HIGH, .5);                    // get away from the lander
        linearOp.sleep(sleepTime);
        myMechDrive.driveForward(.3, .4);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);
        myMechDrive.strafeRight(SPD_DRIVE_HIGH, .51);
        linearOp.sleep(sleepTime);

        myGyro.gyroOrientMecanum(0, myMechDrive);           // different (34)
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);


        switch (goldPosition) {
            case LEFT:
//                myMechDrive.rotateLeft(SPD_DRIVE_MED, .5);         // fixing Gyro issue
//                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(31, myMechDrive);               // different (34)
                myMechDrive.stopMotors();                                       // angle will be wrong
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(31, myMechDrive);               // different (34)
                myMechDrive.stopMotors();                                       // angle will be wrong
                linearOp.sleep(sleepTime);

                myIntakeRotator.mineralRotateLowerEncoder();                     //lower rotater
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArmAllTheWay();                       // extend extender and knock off the mineral
                linearOp.sleep(sleepTime);

//                myMechDrive.rotateRight(SPD_DRIVE_MED, 1.5);             // rotate to align with depot
//                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(0, myMechDrive);                  // corretion of angle with gyro
                myMechDrive.stopMotors();                                       // angle will be wrong
                linearOp.sleep(sleepTime);

//                myGyro.gyroOrientMecanum(0, myMechDrive);                  // corretion of angle with gyro
//                myMechDrive.stopMotors();                                       // angle will be wrong
//                linearOp.sleep(sleepTime);

                myIntakeServo.IntakeServoReverse();                         // spit out the team marker
                linearOp.sleep(servoRotateTeamMarker);

//                myMechDrive.rotateLeft(SPD_DRIVE_MED, .4);             // rotate back to avoid minerals
//                linearOp.sleep(sleepTime);                                       //// angle will be wrong

//                myGyro.gyroOrientMecanum(20, myMechDrive);                  // rotate some more to make sure to not bring the mineral back
//                myMechDrive.stopMotors();
//                linearOp.sleep(sleepTime);
//
//                myGyro.gyroOrientMecanum(30, myMechDrive);                  // rotate some more to make sure to not bring the mineral back
//                myMechDrive.stopMotors();
//                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(40, myMechDrive);                  // rotate some more to make sure to not bring the mineral back
                myMechDrive.stopMotors();
                myIntakeServo.stopIntakeServo();
                linearOp.sleep(sleepTime);

//                myGyro.gyroOrientMecanum(40, myMechDrive);                  // rotate some more to make sure to not bring the mineral back
//                myMechDrive.stopMotors();
//                linearOp.sleep(sleepTime);
//
//                myGyro.gyroOrientMecanum(50, myMechDrive);                  // rotate some more to make sure to not bring the mineral back
//                myMechDrive.stopMotors();
//                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractIntakeArmAuto();                     // retract extender
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(1000);
                myIntakeExtenderArm.stopIntakeArm();
                linearOp.sleep(sleepTime);

                myIntakeRotator.mineralRotateRaiseEncoder();                    // raise rotater
                linearOp.sleep(sleepTime);

                break;


            case MIDDLE:
                myGyro.gyroOrientMecanum(0, myMechDrive);            //turning too much towards the right. Need to adjust?
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

//                myGyro.gyroOrientMecanum(0, myMechDrive);            //turning too much towards the right. Need to adjust?
//                myMechDrive.stopMotors();
//                linearOp.sleep(sleepTime);

                myIntakeRotator.mineralRotateLowerEncoder();                 //lower rotater
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArmAllTheWay();             // extend extender and knock off the mineral
                linearOp.sleep(sleepTime);


                myIntakeServo.IntakeServoReverse();                     // spit out the team marker
                linearOp.sleep(servoRotateTeamMarker);


                myIntakeExtenderArm.retractIntakeArmAuto();                     // retract extender
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);            // another retract to pull the extender all the way back
                linearOp.sleep(1000);

                myIntakeExtenderArm.stopIntakeArm();
                myIntakeServo.stopIntakeServo();
                linearOp.sleep(sleepTime);

                myIntakeRotator.mineralRotateRaiseEncoder();                    // raise rotater
                linearOp.sleep(sleepTime);
                break;

            case RIGHT:

//                myMechDrive.rotateRight(SPD_DRIVE_MED, .5);             // rotate toward mineral
//                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-31, myMechDrive);
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-31, myMechDrive);
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

                myIntakeRotator.mineralRotateLowerEncoder();                    // lower rotater
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArmAllTheWay();                      // extend extender to knock off the mineral
                linearOp.sleep(sleepTime);

//                myMechDrive.rotateLeft(SPD_DRIVE_MED, 1.5);             // rotate toward depot
//                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(0, myMechDrive);
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(0, myMechDrive);
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);

                myIntakeServo.IntakeServoReverse();                         // spit out mineral
                linearOp.sleep(servoRotateTeamMarker);

//                myMechDrive.rotateLeft(SPD_DRIVE_MED, 1.5);            // rotate back to avoid knocking off minerals
//                linearOp.sleep(sleepTime);

                myGyro.gyroOrientMecanum(-40, myMechDrive);
                myMechDrive.stopMotors();
                myIntakeServo.stopIntakeServo();
                linearOp.sleep(sleepTime);

//                myGyro.gyroOrientMecanum(-40, myMechDrive);
//                myMechDrive.stopMotors();
//                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractIntakeArmAuto();                     // retract extender arm
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(1000);
                myIntakeExtenderArm.stopIntakeArm();
                linearOp.sleep(sleepTime);

                myIntakeRotator.mineralRotateRaiseEncoder();                    // raise the rotater
                linearOp.sleep(sleepTime);

                break;
        }
    }


    public void RotateDriveTowardCrater (GyroCompetition myGyro, MecanumDrive myMechDrive) {
        //myMechDrive.driveBackward(SPD_DRIVE_MED,.1);
//        myMechDrive.rotateLeft(.7, SPD_DRIVE_MED);                      // fixing Gyro issue
//        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(71, myMechDrive);                   // gyro towards the crater after dropping tm was -84.9
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(71, myMechDrive);                   // gyro towards the crater after dropping tm was -84.9
        linearOp.sleep(sleepTime);


        myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.5);               // drive toward wall in crater

//        switch (goldPosition) {
//            case LEFT:
//                myGyro.gyroOrientMecanum(85, myMechDrive);
//                break;
//            case MIDDLE:
//                myMechDrive.driveForward(SPD_DRIVE_MED,1.5);                 // drive toward the wall near the crater
//                break;
//            case RIGHT:
//                myGyro.gyroOrientMecanum(94, myMechDrive);
//                myMechDrive.driveForward(SPD_DRIVE_MED,5.5);                 // drive toward the wall near the crater
//                break;
//        }
    }

    public void DriveParkInCrater (GyroCompetition myGyro, MecanumDrive myMechDrive, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo, IntakeRotator myIntakeRotater) {
//        myMechDrive.rotateLeft(SPD_DRIVE_MED,.5);                    // fixing Gyro issue
//        linearOp.sleep(sleepTime);
        myIntakeExtenderArm.retractPowerAuto(1);
        linearOp.sleep(500);
        myIntakeExtenderArm.stopIntakeArm();
        myGyro.gyroOrientMecanum(133, myMechDrive);                 // gyroing at the crater
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(133, myMechDrive);                 // gyroing at the crater
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

//        myMechDrive.strafeRight(SPD_DRIVE_HIGH,.3);
//        linearOp.sleep(sleepTime);

        myMechDrive.setMotorPowerStrafeRight(SPD_DRIVE_HIGH);                      // Align to wall
        linearOp.sleep(1100);                               // Time for straffing
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);
        myMechDrive.setMotorPowerStrafeLeft(SPD_DRIVE_HIGH); //make sure a little off wall so robot does not hit wall seam
        linearOp.sleep(150);
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(137, myMechDrive);                 // gyroing at the crater
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);
//        myMechDrive.driveForward(SPD_DRIVE_HIGH, 1);

//        switch (goldPosition) {
//            case LEFT:
//                myMechDrive.strafeRight(SPD_DRIVE_HIGH, 1.4);
//            case MIDDLE:
//                myMechDrive.strafeRight(SPD_DRIVE_HIGH,.7);
//                myMechDrive.driveForward(SPD_DRIVE_MED, .6);
//            case RIGHT:
//                myMechDrive.driveForward(SPD_DRIVE_MED, .2);
//
//        }

        myIntakeExtenderArm.extendIntakeArmAuto();
        linearOp.sleep(sleepTime);

        myIntakeRotater.mineralRotateLowerEncoder();
//        myIntakeServo.IntakeServoForwardTime();
//        linearOp.sleep(sleepTime);

    }

















    // old methods


//    // *****   Method used by Depot to move from Mineral to Depot   *******  //
//
//    public void mineralToDepot(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance, TeamMarker myTeamMarker) {
//        switch (goldPosition) {
//
//            case LEFT:                                                  // pushes mineral towards wall.  This case will cause mineral to move from depot to crater
//                myMechDrive.driveForward(SPD_DRIVE_LOW, 1);
//                myMechDrive.driveBackward(SPD_DRIVE_LOW, .2);
//
//                myGyro.gyroOrientMecanum(-28, myMechDrive);       //rotates in toward depot
//                myMechDrive.stopMotors();
//
//                myMechDrive.driveForward(SPD_DRIVE_MED, 2.2);
//
//                myGyro.gyroOrientMecanum(88, myMechDrive);        // it was 42  which places the team marker close to tape
//                myMechDrive.stopMotors();                               // Recommend to adjust with adding degrees to turn
//
//                myMechDrive.strafeLeft(.2, .3);           // Straffes so that team marker does not hit glass
//
//                myTeamMarker.teamMarkerArmOutside();                   // drop team marker
//                linearOp.sleep(1000);
//                myTeamMarker.teamMarkerArmRaised();                     // raise arm
//
//                myGyro.gyroOrientMecanum(130, myMechDrive);       //Orient gyro to move from depot to crater
//                myMechDrive.stopMotors();
//                break;
//
//            case MIDDLE:                                                //pushes middle mineral into the depot to score
//                myGyro.gyroOrientMecanum(0, myMechDrive);         // drive forward towards depot just a little bit
//                myMechDrive.stopMotors();
//
//                myMechDrive.driveForward(SPD_DRIVE_MED, 2.1);    // push mineral into depotfor extra point
//                myMechDrive.driveBackward(SPD_DRIVE_LOW, .2);   // back up to clear mineral when turning
//
//                myGyro.gyroOrientMecanum(88, myMechDrive);        // turn to drop team marker was 60 but it was to small of an angle
//                myMechDrive.stopMotors();
//
//                myMechDrive.driveForward(SPD_DRIVE_MED, .3);    // inch forward a little bit to ensure team marker drops in depot
//
//                myTeamMarker.teamMarkerArmOutside();                   // drop team maker
//                linearOp.sleep(1000);
//                myTeamMarker.teamMarkerArmRaised();                     // raise arm
//
//                myGyro.gyroOrientMecanum(130, myMechDrive);       // Orient gyro to move from depot to crater
//                myMechDrive.stopMotors();
//                break;
//
//            case RIGHT:                                                   // pushes mineral into wall
//                myMechDrive.driveForward(SPD_DRIVE_LOW, 1.1);     // Forward to push mineral to clear middle mineral
//                myMechDrive.driveBackward(SPD_DRIVE_LOW, .2);     // backup so turn does not hit mineral
//                myGyro.gyroOrientMecanum(32, myMechDrive);         //  rotates in toward depot
//                myMechDrive.stopMotors();
//
//                myMechDrive.driveForward(SPD_DRIVE_MED, 2.1);     // driving forward into depot
//
//                myGyro.gyroOrientMecanum(88, myMechDrive);         // turn to drop team marker in depot was 60 but it was to small of an angle
//                myMechDrive.stopMotors();
//
//                myMechDrive.strafeLeft(.2, .3);            // straffe so team marker does not hit glass
//
//                myTeamMarker.teamMarkerArmOutside();                     // drop team marker
//                linearOp.sleep(1000);
//                myTeamMarker.teamMarkerArmRaised();                       // raise arm
//
//                myGyro.gyroOrientMecanum(130, myMechDrive);        // Orient gyro to move from depot to crater
//                myMechDrive.stopMotors();
//                break;
//        }
//
//        myMechDrive.stopMotors();
//
//    }
//
////  ***********   Method used for Depot to drive from depot to crater   ******************* //
//
//    public void depotToCrater(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {
//
//
//        switch (goldPosition) {
//
//            case LEFT:
//
//                myMechDrive.setMotorPowerStrafeRight(.5);                //straffe to align with wall
//                linearOp.sleep(1500);
//
//                myMechDrive.driveForward(SPD_DRIVE_MED, 2);     //drive half way toward crater
//                myGyro.gyroOrientMecanum(130, myMechDrive);       // gyro corrects angle around plexiglass seam
//                myMechDrive.stopMotors();
//
//                myMechDrive.driveForward(SPD_DRIVE_MED, 3);     // Shortest distance.... drives the rest of the way
//                break;
//
//            case MIDDLE:
//                myMechDrive.setMotorPowerStrafeRight(.5);                //straffe to align with wall
//                linearOp.sleep(2000);
//
//                myMechDrive.driveForward(SPD_DRIVE_MED, 2);     //drive half way toward crater
//                myGyro.gyroOrientMecanum(130, myMechDrive);       // gyro corrects angle around plexiglass seam
//                myMechDrive.stopMotors();
//
//                myMechDrive.driveForward(SPD_DRIVE_MED, 3.2);   // Medium distance ... drives the rest of the way
//                break;
//
//            case RIGHT:
//                myMechDrive.setMotorPowerStrafeRight(.5);                //straffe to align with wall
//                linearOp.sleep(1500);
//
//                myMechDrive.driveForward(SPD_DRIVE_MED, 2);     //drive half way toward crater
//                myGyro.gyroOrientMecanum(130, myMechDrive);       // gyro corrects angle around plexiglass seam
//                myMechDrive.stopMotors();
//
//                myMechDrive.driveForward(SPD_DRIVE_MED, 3.4);   // Longest distance... drives the rest of the way
//                break;
//        }
//
//    }

  }

