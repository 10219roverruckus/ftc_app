package org.firstinspires.ftc.teamcode.robot.competition.oldClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.competition.autonomous.GoldPosition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotator;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;


public class MecanumMineralMinerCraterOldComments {

    public MecanumMineralMinerCraterOldComments() {

    }
    public GoldPosition goldPosition = null;
    public LinearOpMode linearOp = null;
    public final int RED_THRESHOLD = 30;                //maybe a higher threshold (12.5)
    public final int BLUE_THRESHOLD = 100;              //maybe a higher threshold (12.5)
    float hsvValues[] = {0F, 0F, 0F};

    // created constant variables that are used for speed (different setting)

    final double SPD_DRIVE_LOW = .38;                  //Lowest speed
    final double SPD_DRIVE_MED =  0.5;   //was .4;                   Default is  SPD_MED
    final double SPD_DRIVE_HIGH = .80;
    final double SPD_DRIVE_MAX = 1.0;
    final double SPD_ARM_MED = .5;
    final long sleepTime = 75;
    final int servoRotateTeamMarker = 1000;


    // variables and constants used by color sensor

    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


    public void setLinearOp(LinearOpMode Op) {
        linearOp = Op;
    }




    // new methods for this class

    public void findingMineralCamera(double cameraGoldLocation) {

        //find location of the mineral using camera

        if (cameraGoldLocation < 200 && cameraGoldLocation > 1) {
            goldPosition = GoldPosition.RIGHT;                           //commented out while camera does not work
        } else if (cameraGoldLocation > 200) {                            //program works perfectly DO NOT CHANGE THE CODE
            goldPosition = GoldPosition.MIDDLE;
        } else {
            goldPosition = GoldPosition.LEFT;
        }
//        goldPosition = GoldPosition.LEFT;
    }

    public void driveMineral(GyroCompetition myGyro, MecanumDrive myMechDrive, LiftMotor myLiftMotor, IntakeRotator myIntakeRotator, IntakeExtenderArm myIntakeExtenderArm, IntakeServo myIntakeServo)  {
//        goldPosition = GoldPosition.LEFT;
//        linearOp.telemetry.addData("MINERAL", goldPosition);
//        linearOp.telemetry.update();
//        linearOp.sleep(500);
        // linearOp.sleep(3500);  //SIMULATE LOWING ROBOT - COMMENT OUT WHEN RUNNING LIFT MOTOR!
        myLiftMotor.extendLiftMotorFullyEncoders();                        // using encoders rather than distance sensor
        linearOp.sleep(sleepTime);

        myMechDrive.strafeLeft(SPD_DRIVE_MED, .5);                    // get away from the lander
        linearOp.sleep(sleepTime);
        myMechDrive.driveForward(SPD_DRIVE_HIGH, .3);                  // DRIVES FORWARD SHORT DISTANCE TO GET OFF LANDER
        linearOp.sleep(sleepTime);
        myMechDrive.strafeRight(SPD_DRIVE_HIGH,.51);
        linearOp.sleep(sleepTime);
//        linearOp.telemetry.addData("CASE", goldPosition);
//        linearOp.telemetry.update();
//        linearOp.sleep(1000);
//        linearOp.telemetry.addLine("GO TO CASE");
//        linearOp.telemetry.update();
        switch (goldPosition) {                                            //Gyro angles robot to push off mineral
            case LEFT:
//                linearOp.telemetry.addLine("LEFT CASE");
//                linearOp.telemetry.update();
//                linearOp.telemetry.addLine("ROTATE LEFT");
                linearOp.telemetry.update();
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, .5);          // fixing Gyro issue
                linearOp.sleep(sleepTime);
//                linearOp.telemetry.addLine("ANGLE LEFT");
                linearOp.telemetry.update();
                myGyro.gyroOrientMecanum(30, myMechDrive);           // different (34)
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
//                linearOp.telemetry.addLine("MINERAL THING ");
//                linearOp.telemetry.update();

                myIntakeRotator.mineralRotateLowerEncoder();
                linearOp.sleep(sleepTime);
//
//                myMechDrive.driveForward(SPD_DRIVE_MED,1);
//                linearOp.sleep(sleepTime);
//                myMechDrive.driveBackward(SPD_DRIVE_MED,.4);

//                linearOp.sleep(sleepTime);
//                myIntakeExtenderArm.retractPowerAuto(1);
//                linearOp.sleep(500);
//                myIntakeExtenderArm.stopIntakeArm();




                myIntakeExtenderArm.extendIntakeArmAuto();
                linearOp.sleep(sleepTime);
                myIntakeExtenderArm.retractIntakeArmAuto();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.mineralRotateRaiseEncoder();
                linearOp.sleep(sleepTime);
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 0.7);              // fixing Gyro issue of over rotating

                break;

            case MIDDLE:
//                linearOp.telemetry.addData("MIDDLE", goldPosition);
//                linearOp.telemetry.update();
//                myMechDrive.rotateLeft(SPD_DRIVE_MED,.1);
//                linearOp.sleep(sleepTime);
                myGyro.gyroOrientMecanum(0, myMechDrive);            //turning too much towards the right. Need to adjust?
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeRotator.mineralRotateLowerEncoder();
                linearOp.sleep(sleepTime);

//                myMechDrive.driveForward(SPD_DRIVE_MED,1);
//                linearOp.sleep(sleepTime);
//                myMechDrive.driveBackward(SPD_DRIVE_MED,.5);
//                linearOp.sleep(sleepTime);
//                myIntakeExtenderArm.intakeExtenderArm.setPower(1);
//                linearOp.sleep(500);
//                myIntakeExtenderArm.stopIntakeArm();


                myIntakeExtenderArm.extendTowardMiddleMineral();
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.retractIntakeArmAuto();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.mineralRotateRaiseEncoder();
                linearOp.sleep(sleepTime);
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 1.3);              // fixing Gyro issue of over rotating
                linearOp.sleep(sleepTime);


                break;

            case RIGHT:
//                linearOp.telemetry.addData("RIGHT", goldPosition);
//                linearOp.telemetry.update();
                myMechDrive.rotateRight(SPD_DRIVE_HIGH, .5);         // fixing Gyro issue
                linearOp.sleep(sleepTime);
                myGyro.gyroOrientMecanum(-32, myMechDrive);          // Gyro angles appears correct. was -14
                myMechDrive.stopMotors();
                linearOp.sleep(sleepTime);
                myIntakeRotator.mineralRotateLowerEncoder();
                linearOp.sleep(sleepTime);

//                myMechDrive.driveForward(SPD_DRIVE_MED,1.2);
//                linearOp.sleep(sleepTime);
//                myMechDrive.driveBackward(SPD_DRIVE_MED,.5);
//                linearOp.sleep(sleepTime);

//                myIntakeExtenderArm.intakeExtenderArm.setPower(1);
//                linearOp.sleep(500);
//                myIntakeExtenderArm.stopIntakeArm();

                myIntakeExtenderArm.extendIntakeArmAuto();
                linearOp.sleep(sleepTime);


                myIntakeExtenderArm.retractIntakeArmAuto();
                linearOp.sleep(sleepTime);

                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();

                myIntakeRotator.mineralRotateRaiseEncoder();
                linearOp.sleep(sleepTime);
                myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 1.9);              // fixing Gyro issue of over rotating was 1.9
                break;
        }
    }







    public void RotateDriveWall (GyroCompetition myGyro, MecanumDrive myMechDrive, IntakeExtenderArm myIntakeExtenderArm) {

//        myMechDrive.rotateLeft(SPD_DRIVE_MED, 1.3);              // fixing Gyro issue of over rotating
//        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(74, myMechDrive);                //orients self with red tape so parallel to tape.
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        myMechDrive.strafeRight(SPD_DRIVE_MED,.55);  // to get away from lander - too far = hit mineral, not enough = hit lander going to wall

        switch (goldPosition) {
            case LEFT:
                myMechDrive.driveForward(SPD_DRIVE_HIGH,2.2);
                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();
                break;
            case MIDDLE:
                myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.3);
                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();
                break;
            case RIGHT:
                myMechDrive.driveForward(SPD_DRIVE_HIGH, 2.4);
                myIntakeExtenderArm.retractPowerAuto(1);
                linearOp.sleep(500);
                myIntakeExtenderArm.stopIntakeArm();
                break;
        }


        linearOp.sleep(sleepTime);
//        myGyro.gyroOrientMecanum(74, myMechDrive);                //orients self with red tape so parallel to tape.
//        myMechDrive.stopMotors();
    }




    public void RotateDriveTowardDepot(GyroCompetition myGyro, MecanumDrive myMechDrive) {
        myMechDrive.rotateLeft(SPD_DRIVE_MED, 1);              // fixing Gyro issue
        linearOp.sleep(sleepTime);
        myGyro.gyroOrientMecanum(137, myMechDrive);              // Orient for straight drive to depot
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);

        myMechDrive.setMotorPowerStrafeRight(SPD_DRIVE_HIGH);                      // Align to wall
        linearOp.sleep(800);                               // Time for straffing
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);
        myMechDrive.setMotorPowerStrafeLeft(SPD_DRIVE_HIGH); //make sure a little off wall so robot does not hit wall seam  was med
        linearOp.sleep(100);
        myMechDrive.stopMotors();
        linearOp.sleep(sleepTime);

        myGyro.gyroOrientMecanum(138, myMechDrive);              // Orient for straight drive to depot was 137
        myMechDrive.stopMotors();                                      // Stop motors
        linearOp.sleep(sleepTime);
        myMechDrive.driveForward(SPD_DRIVE_HIGH,.6);
//
//        myMechDrive.driveForward(SPD_DRIVE_HIGH, 1.0);           //going toward depot using color sensor
//        linearOp.sleep(sleepTime);
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
        linearOp.sleep(servoRotateTeamMarker);

        myIntakeExtenderArm.retractIntakeArmAuto();
        linearOp.sleep(sleepTime);

        myIntakeExtenderArm.retractPowerAuto(1);
        linearOp.sleep(1000);
        myIntakeExtenderArm.stopIntakeArm();
        myIntakeServo.stopIntakeServo();

        myIntakeRotator.mineralRotateRaiseEncoder();
        linearOp.sleep(sleepTime);

    }




    public void DriveParkInCrater (MecanumDrive myMechDrive, IntakeExtenderArm myIntakeExtender, IntakeRotator myIntakeRotater) {
        // drive backward and park in crater

        myMechDrive.driveBackward(1, .9);
        linearOp.sleep(sleepTime);

        myMechDrive.setMotorPowerStrafeRight(SPD_DRIVE_HIGH);       // was med
        linearOp.sleep(200);
        myMechDrive.stopMotors();

        myMechDrive.driveBackward(.35, .5);         // was 1.1
        linearOp.sleep(sleepTime);

//        myMechDrive.rotateLeft(SPD_DRIVE_HIGH, 3.1);        // was 1.7
//        linearOp.sleep(sleepTime);

//        myMechDrive.setMotorPowerStrafeLeft(SPD_DRIVE_HIGH);
//        linearOp.sleep(200);
//        myMechDrive.stopMotors();

//        myMechDrive.driveForward(SPD_DRIVE_HIGH, .3);

//        myIntakeExtender.extendIntakeArmAuto();
//        linearOp.sleep(sleepTime);
//
//        myIntakeRotater.mineralRotateLowerEncoder();

        myMechDrive.driveBackward(.28, .3);
        linearOp.sleep(sleepTime);
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

