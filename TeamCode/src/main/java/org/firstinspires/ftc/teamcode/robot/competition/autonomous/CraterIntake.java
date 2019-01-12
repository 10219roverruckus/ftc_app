package org.firstinspires.ftc.teamcode.robot.competition.autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.GyroCompetition;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.constructor.sensors.RevColorDistance;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeExtenderArm;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeRotator;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.IntakeServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LanderServo;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.LiftMotor;
import org.firstinspires.ftc.teamcode.robot.competition.mechanisms.motors.TeamMarker;
import org.firstinspires.ftc.teamcode.robot.competition.oldClasses.MecanumMineralMiner;
import org.firstinspires.ftc.teamcode.robot.competition.autonomous.MecanumMineralMinerAll;
import org.firstinspires.ftc.teamcode.robot.competition.autonomous.MecanumMineralMinerCrater;
import org.firstinspires.ftc.teamcode.robot.competition.autonomous.MecanumMineralMinerDepot;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


@Autonomous(name = "Crater - Competition Intake")
//@Disabled
public class CraterIntake extends LinearOpMode  {

    MecanumDrive myMechDrive;

    GyroCompetition myGyro;
    MecanumMineralMiner myMineralMiner;
//    RevColorDistance myRevColorDistance;

    MecanumMineralMinerCrater myMineralMinerCrater;
    MecanumMineralMinerDepot myMineralMinerDepot;
    MecanumMineralMinerAll myMineralMinerAll;

    LiftMotor myLiftMotor;
    IntakeExtenderArm myIntakeExtenderArm;
    IntakeRotator myIntakeRotator;
    IntakeServo myIntakeServo;
    LanderServo myLanderServo;


    private GoldAlignDetector detector;
    WebcamName webcamName;
    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    Dogeforia vuforia;
    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    @Override
    public void runOpMode() throws InterruptedException {



        final long sleepTime = 100;
        final double SPD_DRIVE_MED = .5;


        myMechDrive = new MecanumDrive(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        myMechDrive.setLinearOp(this);

        myGyro = new GyroCompetition(hardwareMap.get(BNO055IMU.class, "imu"));
        myGyro.setLinearOp(this);

        myMineralMiner = new MecanumMineralMiner();
        myMineralMiner.setLinearOp(this);

        myLiftMotor = new LiftMotor(hardwareMap.dcMotor.get("lift_motor"));
        myLiftMotor.setLinearOp(this);

        myIntakeRotator = new IntakeRotator(hardwareMap.dcMotor.get("intake_rotater_motor"));
        myIntakeRotator.setLinearOp(this);

        myIntakeServo = new IntakeServo(hardwareMap.servo.get("intake_spinner_servo_left"), hardwareMap.servo.get("intake_spinner_servo_right"));
        myIntakeServo.setLinearOp(this);

        myIntakeExtenderArm  = new IntakeExtenderArm (hardwareMap.dcMotor.get("intake_extender_arm"));
        myIntakeExtenderArm.setLinearOp(this);

//        myRevColorDistance = new RevColorDistance(hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance"), hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance_mineral_lift"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance_mineral_lift"), hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance_hook"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance_hook"), hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance_extender"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance_extender"));
//        myRevColorDistance.setLinearOp(this);

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        myMineralMinerCrater = new MecanumMineralMinerCrater();
        myMineralMinerCrater.setLinearOp(this);

        myMineralMinerDepot = new MecanumMineralMinerDepot();
        myMineralMinerDepot.setLinearOp(this);

        myMineralMinerAll = new MecanumMineralMinerAll();
        myMineralMinerAll.setLinearOp(this);

        myLanderServo = new LanderServo (hardwareMap.servo.get("mineral_dumper"), hardwareMap.servo.get("transfer_gate_servo"));
        myLanderServo.setLinearOp(this);
        myLanderServo.landerServoCollect();
        myLanderServo.keepMineralsIn();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AWbfTmn/////AAABmY0xuIe3C0RHvL3XuzRxyEmOT2OekXBSbqN2jot1si3OGBObwWadfitJR/D6Vk8VEBiW0HG2Q8UAEd0//OliF9aWCRmyDJ1mMqKCJZxpZemfT5ELFuWnJIZWUkKyjQfDNe2RIaAh0ermSxF4Bq77IDFirgggdYJoRIyi2Ys7Gl9lD/tSonV8OnldIN/Ove4/MtEBJTKHqjUEjC5U2khV+26AqkeqbxhFTNiIMl0LcmSSfugGhmWFGFtuPtp/+flPBRGoBO+tSl9P2sV4mSUBE/WrpHqB0Jd/tAmeNvbtgQXtZEGYc/9NszwRLVNl9k13vrBcgsiNxs2UY5xAvA4Wb6LN7Yu+tChwc+qBiVKAQe09\n";
        parameters.fillCameraMonitorViewParent = true;

        parameters.cameraName = webcamName;

        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */

        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        targetsRoverRuckus.activate();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.downscale = 0.8;

        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();


        waitForStart();

        idle();

        detector.goldXPos = 0;                                                              // sets gold position to zero, so the camera does not guess the position
        sleep(100);

        myMineralMinerCrater.findingMineralCamera(detector.getXPosition());                      // detect gold position

        sleep(sleepTime);
        idle();

        myMineralMinerCrater.driveMineral(myGyro, myMechDrive, myLiftMotor,  myIntakeRotator, myIntakeExtenderArm, myIntakeServo);                     // push gold off of little square

        sleep(sleepTime);
        idle();

        myMineralMinerCrater.RotateDriveWall (myGyro, myMechDrive, myIntakeExtenderArm);      // Backups to tape under Lander and moves towards wall

        sleep(sleepTime);
        idle();

        myMineralMinerCrater.RotateDriveTowardDepot(myGyro, myMechDrive);  // Aligns to Wall, Drives to Depot, Drops off Mineral, and drives back to Crater

        sleep(sleepTime);
        idle();

        myMineralMinerCrater.LowerReleaseTM(myIntakeExtenderArm, myIntakeRotator, myIntakeServo );

        sleep(sleepTime);
        idle();

        myMineralMinerCrater.DriveParkInCrater(myMechDrive);
        sleep(sleepTime);
        idle();
    }
}