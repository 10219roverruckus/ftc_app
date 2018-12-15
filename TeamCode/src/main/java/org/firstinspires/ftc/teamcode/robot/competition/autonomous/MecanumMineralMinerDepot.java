package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotator;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.TeamMarker;


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
    final long sleepTime = 0;

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


    public void driveMineral(GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotator myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm) {

        linearOp.telemetry.addData("MINERAL", goldPosition);
        linearOp.telemetry.update();
        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor

        myMechDrive.strafeRight(.3, .3);                    // get away from the lander
        myMechDrive.driveForward(.3, .3);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER


        switch (goldPosition) {                                            //Gyro angles robot to push off mineral
            case LEFT:

                myMechDrive.strafeRight(SPD_DRIVE_MED, 1.5);

                myMechDrive.rotateLeft(SPD_DRIVE_MED, .5);         // fixing Gyro issue
                myGyro.gyroOrientMecanum(36, myMechDrive);           // different (34)

                myMechDrive.stopMotors();
                myIntakeRotator.mineralRotateLowerEncoder();
                myIntakeExtenderArm.extendIntakeArmAuto();

                myIntakeExtenderArm.retractIntakeArmAuto();
                myIntakeRotator.mineralRotateRaiseEncoder();
                //        myMechDrive.driveForward(SPD_DRIVE_MED, 2.15);    // Moves forward to push off mineral (Originally 1.8)
                break;

            case MIDDLE:
                myMechDrive.strafeRight(SPD_DRIVE_MED, .3);

                myGyro.gyroOrientMecanum(4, myMechDrive);            //turning too much towards the right. Need to adjust?
                myMechDrive.stopMotors();
                myIntakeRotator.mineralRotateLowerEncoder();
                myIntakeExtenderArm.extendIntakeArmAuto();

                myIntakeExtenderArm.retractIntakeArmAuto();
                myIntakeRotator.mineralRotateRaiseEncoder();
                //        myMechDrive.driveForward(SPD_DRIVE_MED, 1.7);      // Moves forward to push off mineral
                break;

            case RIGHT:

                myMechDrive.strafeLeft(SPD_DRIVE_MED, .5);

                myMechDrive.rotateRight(SPD_DRIVE_MED, .3);        // fixing Gyro issue
                myGyro.gyroOrientMecanum(-14, myMechDrive);          // Gyro angles appears correct.
                myMechDrive.stopMotors();
                myIntakeRotator.mineralRotateLowerEncoder();
                myIntakeExtenderArm.extendIntakeArmAuto();

                myIntakeExtenderArm.retractIntakeArmAuto();
                myIntakeRotator.mineralRotateRaiseEncoder();
                //        myMechDrive.driveForward(SPD_DRIVE_MED, 2);        // Moves forward to push off mineral (Originally 1.8)
                break;
        }
    }


    public void RotateDriveTowardCrater (GyroCompetition myGyro, MecanumDrive myMechDrive) {
        myMechDrive.rotateLeft(.7, SPD_DRIVE_MED);                      // fixing Gyro issue
        myGyro.gyroOrientMecanum(-84.2, myMechDrive);                   // gyro towards the crater after dropping tm

        myMechDrive.driveForward(SPD_DRIVE_MED,2);                  // drive toward the wall near the crater

    }

    public void DriveParkInCrater (GyroCompetition myGyro, MecanumDrive myMechDrive) {
        myMechDrive.strafeRight(SPD_DRIVE_LOW, .2);

        myMechDrive.rotateLeft(SPD_DRIVE_MED,.5);                    // fixing Gyro issue
        myGyro.gyroOrientMecanum(-134.6, myMechDrive);                 // gyroing at the crater
        myMechDrive.driveForward(SPD_DRIVE_MED, .5);                // drive a little to break the plane of the crater
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

