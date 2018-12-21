package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotator;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;


public class MecanumMineralMinerCrater {

    public MecanumMineralMinerCrater() {

    }
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

    // variables and constants used by color sensor

    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }




    // new methods for this class


    public void driveMineral(GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotator myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo)  {

        linearOp.telemetry.addData("MINERAL", goldPosition);
        linearOp.telemetry.update();
        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor
        linearOp.sleep(sleepTime);
        myMechDrive.strafeRightPID(SPD_DRIVE_HIGH, .3);                    // get away from the lander
        linearOp.sleep(sleepTime);
        myMechDrive.driveForwardPID(SPD_DRIVE_HIGH, .3);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);
        myMechDrive.strafeLeftPID(SPD_DRIVE_HIGH,.4);

        switch (goldPosition) {                                            //Gyro angles robot to push off mineral
            case LEFT:
                myMechDrive.rotateLeft(SPD_DRIVE_MED, .4);          // fixing Gyro issue
                linearOp.sleep(sleepTime);
                myGyro.gyroOrientMecanum(36, myMechDrive);           // different (34)
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeRotator.mineralRotateLowerEncoder();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArmAuto();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractIntakeArmAuto();
                linearOp.sleep(sleepTime);

                myIntakeRotator.mineralRotateRaiseEncoder();
                linearOp.sleep(sleepTime);

                break;

            case MIDDLE:
                myMechDrive.rotateLeft(SPD_DRIVE_MED,.1);
                linearOp.sleep(sleepTime);
                myGyro.gyroOrientMecanum(4, myMechDrive);            //turning too much towards the right. Need to adjust?
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeRotator.mineralRotateLowerEncoder();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArmAuto();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractIntakeArmAuto();
                linearOp.sleep(sleepTime);

                myIntakeRotator.mineralRotateRaiseEncoder();
                linearOp.sleep(sleepTime);

                break;

            case RIGHT:
                myMechDrive.rotateRight(SPD_DRIVE_MED, .3);         // fixing Gyro issue
                linearOp.sleep(sleepTime);
                myGyro.gyroOrientMecanum(-14, myMechDrive);          // Gyro angles appears correct.
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeRotator.mineralRotateLowerEncoder();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.extendIntakeArmAuto();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractIntakeArmAuto();
                linearOp.sleep(sleepTime);

                myIntakeRotator.mineralRotateRaiseEncoder();
                linearOp.sleep(sleepTime);

                break;
        }
    }







    public void RotateDriveWall (GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {

        myMechDrive.rotateLeft(SPD_DRIVE_MED, .7);              // fixing Gyro issue of over rotating
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(74, myMechDrive);                //orients self with red tape so parallel to tape.
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        myMechDrive.driveForwardPID(SPD_DRIVE_HIGH, 3.7);
        linearOp.sleep(sleepTime);
    }




    public void RotateDriveTowardDepot(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {
        myMechDrive.rotateLeft(SPD_DRIVE_MED, .4);              // fixing Gyro issue
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(137, myMechDrive);              // Orient for straight drive to depot
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);

        myMechDrive.setMotorPowerStrafeRight(.3);                      // Align to wall
        linearOp.sleep(500);                               // Time for straffing
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);

        myMechDrive.driveForwardPID(SPD_DRIVE_HIGH, 2.5);           //going toward depot using color sensor
        linearOp.sleep(sleepTime);
    }





    public void LowerReleaseTM ( IntakeExtenderArm myIntakeExtenderArm, IntakeRotator myIntakeRotator, IntakeServo myIntakeServo) {
        // extend arm and lower rotator
        // rotator will spin to release TM
        // extender will retract and rotator will raise

        myIntakeRotator.mineralRotateLowerEncoder();
        linearOp.sleep(sleepTime);
        myIntakeExtenderArm.extendIntakeArmAuto();
        linearOp.sleep(sleepTime);

        myIntakeServo.IntakeServoReverse();
        linearOp.sleep(sleepTime);

        myIntakeExtenderArm.retractIntakeArmAuto();
        linearOp.sleep(sleepTime);
        myIntakeRotator.mineralRotateRaiseEncoder();
        linearOp.sleep(sleepTime);

    }




    public void DriveParkInCrater (MecanumDrive myMechDrive) {
        // drive backward and park in crater

        myMechDrive.driveBackwardPID(SPD_DRIVE_HIGH, 2.5);
    }
















    // old methods


//    public void craterMineralToWall(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance) {
//
//        myMechDrive.stopMotors();
//
//        Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),     // Move backwards until color detected
//                (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
//                (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
//                hsvValues);
//        while (hsvValues[0] > RED_THRESHOLD && hsvValues[0] < BLUE_THRESHOLD) {
//            Color.RGBToHSV((int) (myRevColorDisance.revColorSensor.red() * SCALE_FACTOR),
//                    (int) (myRevColorDisance.revColorSensor.green() * SCALE_FACTOR),
//                    (int) (myRevColorDisance.revColorSensor.blue() * SCALE_FACTOR),
//                    hsvValues);
//            myMechDrive.setMotorSpeeds(-SPD_DRIVE_MED);
//            //myMechDrive.setMotorSpeeds(SPD_DRIVE_MED);
//            linearOp.idle();
//        }
//        myMechDrive.stopMotors();                                       //DRIVE FUNCTION DOESN'T HAVE A STOP.MOTORS IN IT
//
//        myGyro.gyroOrientMecanum(74, myMechDrive);                //orients self with red tape so parallel to tape.
//        myMechDrive.stopMotors();
//
//        switch (goldPosition) {                                          //drive toward wall distance is different based on distance
//            case LEFT:
//                myMechDrive.driveForward(SPD_DRIVE_MED, 3.6);   // different distance to wall after backup to tape DO NOT CHANGE
//                break;
//            case MIDDLE:
//                myMechDrive.driveForward(SPD_DRIVE_MED, 3.7);  // different distance to wall after backup to tape DO NOT CHANGE
//                break;
//            case RIGHT:
//                myMechDrive.driveForward(SPD_DRIVE_MED, 3.9);  // different distance to wall after backup to tape used to be 4.2 but was too long DO NOT CHANGE
//                break;
//        }
//    }
//
//    // *****   Method used for Crater to drive from along wall to Depot  ********//
//
//    public void wallToDepot(GyroCompetition myGyro, MecanumDrive myMechDrive, RevColorDistance myRevColorDisance, TeamMarker myTeamMarker) {
//
//        myGyro.gyroOrientMecanum(137, myMechDrive);              // Orient for straight drive to depot
//        myMechDrive.stopMotors();                                      // Stop motors
//
//        myMechDrive.setMotorPowerStrafeRight(.3);                      // Align to wall
//        linearOp.sleep(1500);                               // Time for straffing
//        myMechDrive.stopMotors();                                      // Stop motors
//
//        myMechDrive.driveForward(SPD_DRIVE_MED, 3);           //going toward depot using color sensor
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
//            myMechDrive.setMotorSpeeds(SPD_DRIVE_MED);
//
//            linearOp.idle();
//        }
//
//        myMechDrive.stopMotors();                                 // Robot is now in Depot
//
//        myGyro.gyroOrientMecanum(170, myMechDrive);         //rotate to drop team marker into depot
//        myMechDrive.stopMotors();                                 // stop motors
//
//        myMechDrive.strafeLeft(.2, .3);            // strafe away so the marker does not get stuck on wall
//
//        myTeamMarker.teamMarkerArmOutside();                      // drop team maker
//        linearOp.sleep(1250);
//        myTeamMarker.teamMarkerArmRaised();
//        linearOp.sleep(500);
//
//        myMechDrive.strafeLeft(SPD_DRIVE_LOW, .2);         // get away from team maker to it does not get caught on the wheel
//        myMechDrive.driveBackward(SPD_DRIVE_LOW, .7);
//
//
//        myMechDrive.stopMotors();
//
//        myGyro.gyroOrientMecanum(137, myMechDrive);         // Orient straight to park in crater... Angle between 136 - 139
//        myMechDrive.stopMotors();                                 // 138 degrees forces us into the plexiglass
//        linearOp.sleep(500);
//        linearOp.idle();
//
//        myMechDrive.setMotorPowerStrafeRight(.3);                 // staffing into wall
//        linearOp.sleep(1000);
//
//        myMechDrive.driveBackward(SPD_DRIVE_MED, 2.0);    // Drive to park in crater
//
//        myGyro.gyroOrientMecanum(137, myMechDrive);         // Gyro correction for plexiglass. Same angle as above.
//        myMechDrive.stopMotors();
//        linearOp.sleep(500);
//
//        myMechDrive.driveBackward(SPD_DRIVE_MED, 3.3);    //Drive past plexiglass seam
//
//    }




}

