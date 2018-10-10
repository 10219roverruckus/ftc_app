package org.firstinspires.ftc.teamcode.robot.old;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.boardArm;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.colorSensorArm;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.glyphArms;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.glyphLift;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.relicArm;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.revColorDistanceSensor;
import org.firstinspires.ftc.teamcode.robot.old.classes.sub.mechDriveAuto;

/**
 * Created by johnduval on 10/7/17.
 */

@Autonomous(name = "Blue - 2", group = "BLUE")
@Disabled
public class Aut_Blue_2 extends LinearOpMode {
    int movement = 0; //switch variable to determine movementt

    colorSensorArm myColorSensorArm;
    mechDriveAuto myMechDrive;
    glyphArms myGlyphArms;
    glyphLift myGlyphLift;
    boardArm myBoardArm;
    revColorDistanceSensor myRevColorDistanceSensor;
    relicArm myRelicArm;

    boolean distanceSensorInRange;

    //info for gyro
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    //float heading;

    // 1 == LEFT
    // 2 == CENTER & DEFAULT
    // 3 == RIGHT
    int cryptoboxColumn; // determinds column for robot to aim for based on the vuforia key.

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;
    @Override
    public void runOpMode() throws InterruptedException {

        myGlyphLift = new glyphLift(hardwareMap.dcMotor.get("glyph_lift"));
        myColorSensorArm = new colorSensorArm(hardwareMap.servo.get("color_sensor_arm"),hardwareMap.colorSensor.get("rev_color_sensor_arm"), hardwareMap.servo.get("color_sensor_arm_rotate"));
        myMechDrive = new mechDriveAuto(hardwareMap.dcMotor.get("front_left_motor"), hardwareMap.dcMotor.get("front_right_motor"), hardwareMap.dcMotor.get("rear_left_motor"), hardwareMap.dcMotor.get("rear_right_motor"));
        myGlyphArms = new glyphArms(hardwareMap.servo.get("top_left_glyph_arm"), hardwareMap.servo.get("bottom_left_glyph_arm"), hardwareMap.servo.get("top_right_glyph_arm"), hardwareMap.servo.get("bottom_right_glyph_arm"));
        myBoardArm = new boardArm(hardwareMap.dcMotor.get("board_arm"));
        myRevColorDistanceSensor =  new revColorDistanceSensor(hardwareMap.get(ColorSensor.class, "rev_sensor_color_distance"), hardwareMap.get(DistanceSensor.class, "rev_sensor_color_distance"));
        myRelicArm = new relicArm(hardwareMap.dcMotor.get("relic_arm_lift"), hardwareMap.dcMotor.get("relic_arm_extension"), hardwareMap.servo.get("relic_arm_grabber"));

        myColorSensorArm.colorSensorArmUpSlow();
        myColorSensorArm.colorRotateResting();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ASmjss3/////AAAAGQGMjs1d6UMZvrjQPX7J14B0s7kN+rWOyxwitoTy9i0qV7D+YGPfPeeoe/RgJjgMLabjIyRXYmDFLlJYTJvG9ez4GQSI4L8BgkCZkUWpAguRsP8Ah/i6dXIz/vVR/VZxVTR5ItyCovcRY+SPz3CP1tNag253qwl7E900NaEfFh6v/DalkEDppFevUDOB/WuLZmHu53M+xx7E3x35VW86glGKnxDLzcd9wS1wK5QhfbPOExe97azxVOVER8zNNF7LP7B+Qeticfs3O9pGXzI8lj3zClut/7aDVwZ10IPVk4oma6CO8FM5UtNLSb3sicoKV5QGiNmxbbOlnPxz9qD38UAHshq2/y0ZjI/a8oT+doCr";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        BNO055IMU.Parameters parametersimu = new BNO055IMU.Parameters();
        parametersimu.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parametersimu.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parametersimu.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parametersimu.loggingEnabled = true;
        parametersimu.loggingTag = "IMU";
        parametersimu.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

// Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
// on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
// and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parametersimu);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            switch (movement) {
                case 0:
                    telemetry.addData("CASE: ", movement);
                    telemetry.update();
                    myRelicArm.relicGrabberOpen();
                    myRelicArm.setLiftPower(-1);
                    sleep(1300);
                    myRelicArm.setLiftPower(0);
                    myGlyphArms.openLoweredGlyphArms();
                    sleep(450);
                    myGlyphArms.closeGlyphArms();
                    sleep(200);
                    myGlyphLift.raiseGlyphLiftAutMode();
                    movement ++; //move on to next movement
                    break;
                case 1: // reading Vuforia code
                    telemetry.addData("CASE: ", movement);
                    sleep(1500);
                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                        if (pose != null) {
                            VectorF trans = pose.getTranslation();
                            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                            double tX = trans.get(0);
                            double tY = trans.get(1);
                            double tZ = trans.get(2);

                            double rX = rot.firstAngle;
                            double rY = rot.secondAngle;
                            double rZ = rot.thirdAngle;
                        }
                    }
                    else {
                        telemetry.addData("VuMark", "not visible");
                    }
                    switch (vuMark) {
                        case LEFT:
                            cryptoboxColumn = 1;
                            telemetry.addData("LEFT SWITCH - Column: ", cryptoboxColumn);
                            break;
                        case CENTER:
                            cryptoboxColumn = 2;
                            telemetry.addData("CENTER SWITCH - Column: ", cryptoboxColumn);
                            break;
                        case RIGHT:
                            cryptoboxColumn = 3;
                            telemetry.addData("RIGHT SWITCH - Column: ", cryptoboxColumn);
                            break;
                        default:
                            cryptoboxColumn = 2;
                            telemetry.addData("DEFAULT SWITCH - Column: ", cryptoboxColumn);
                            break;
                    }
                    telemetry.update();
                    movement++;
                    break;
                case 2: //detecting jewel and knocking off & centering
                    telemetry.addLine("MOVING SERVO ARM DOWN");
                    telemetry.update();
                    myColorSensorArm.colorSensorArmDownSlow();
                    telemetry.addData("CASE: ", movement);
                    telemetry.addData("Servo", "Position: " + String.format("%.3f", myColorSensorArm.colorSensorArm.getPosition()));
                    telemetry.addData("BLUE: ", myColorSensorArm.colorSensor.blue());
                    telemetry.addData("RED: ", myColorSensorArm.colorSensor.red());
                    telemetry.update();
                    sleep(1000);
                    //robot will move dependeing on the color sensed in myColorArm.colorJewel()
                    //colorJewel passes an int to redAllianceJewel so knows which direction to move
                    //1 = red jewel on left and strafe right
                    //2 = blue jewel on leeft and strafe left
                    //3 = no color detected and do no strafe at all
                    myMechDrive.blueJewel(myColorSensorArm, myColorSensorArm.colorJewel());
                    movement ++;
                    break;
                case 3: //rotate left
                    telemetry.addData("CASE: ", movement);
                    telemetry.update();
                    myMechDrive.encoderDrive(21, 5, 0.30);
                    sleep(200);
                    movement ++;
                    break;
                case 4: //FORWARD Off Platform
                    myMechDrive.encoderDrivePlatform(21.5,.35); // drives off platform using RUN_USING_ENCODERS - distance will vary!
                    sleep(200);
                    movement++;
                    break;
                case 5: //use gyro to orient robot
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    telemetry.addData("CASE gyro: ", movement);
                    telemetry.addData("MOVING","");
                    telemetry.addData("Gyro Heading: ", angles.firstAngle);
                    telemetry.update();
                    sleep(1000);
                    if (angles.firstAngle <= 89.5) {  //robot did NOT rotate enough coming off platform
                        while (angles.firstAngle <= 89.5) {
                            myMechDrive.powerDrive(5, .15);
                            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        }
                    }
                    else if (angles.firstAngle >= 90.5) {    //robot rotated TOO MUCH coming off platform
                        while (angles.firstAngle >= 90.5) {
                            myMechDrive.powerDrive(6,.15);
                            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        }
                    }
                    myMechDrive.stopMotors();
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("DONE MOVING","");
                    telemetry.addData("Gyro Heading: ", angles.firstAngle);
                    telemetry.update();
                    sleep(250);
                    movement++;
                    break;
                case 6: //STRAFE RIGHT IN ORIENTATION WITH CRYPTOBOX
                    myMechDrive.encoderDriveMat(16,4,.3);
                    sleep(200);
                    movement ++;
                    break;
                case 7: // gyro - am I straight?
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    gravity = imu.getGravity();
                    telemetry.addData("CASE gyro: ", movement);
                    telemetry.addData("MOVING","");
                    telemetry.addData("Gyro Heading: ", angles.firstAngle);
                    telemetry.update();
                    sleep(1000);
                    if (angles.firstAngle <= 89.5) {  //robot did NOT rotate enough coming off platform
                        while (angles.firstAngle <= 89.5) {
                            myMechDrive.powerDrive(5, .15);
                            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        }
                    }
                    else if (angles.firstAngle >= 90.5) {    //robot rotated TOO MUCH coming off platform
                        while (angles.firstAngle >= 90.5) {
                            myMechDrive.powerDrive(6,.15);
                            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        }
                    }
                    myMechDrive.stopMotors();
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                    telemetry.addData("DONE MOVING","");
                    telemetry.addData("Gyro Heading: ", angles.firstAngle);
                    telemetry.update();
                    sleep(250);
                    movement++;
                    break;
                case 8: //GO FORWARD TO CRYPTOBOX
                    myGlyphLift.lowerGlyphLiftAutMode();
                    //from case 1 where we get the vuforia code
                    // 1 == LEFT
                    // 2 == CENTER & DEFAULT
                    // 3 == RIGHT
                    switch (cryptoboxColumn) {
                        case 1:
                            myMechDrive.vuforiaLeftPower2(myGlyphArms);
                            break;
                        case 2:
                            myMechDrive.vuforiaCenterPower2(myGlyphArms);
                            break;
                        case 3:
                            myMechDrive.vuforiaRightPower2(myGlyphArms);
                            break;
                    }
                    movement ++;
                    break;
                case 9:
                    requestOpModeStop();
                    break;
            }
            telemetry.addData("Servo", "Position: " + String.format("%.3f", myColorSensorArm.colorSensorArm.getPosition()));
            telemetry.update();
        }
    }
}
