/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.team535;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareTOBOR
{
    /* Public OpMode members. */
    DcMotor FRMotor;
    DcMotor FLMotor;
    DcMotor BRMotor;
    DcMotor BLMotor;
    DcMotor rightTrack;
    DcMotor leftTrack;
    Servo RPlate;
    Servo LPlate;

    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    RelicRecoveryVuMark vuMark;
    TOBORVuMarkIdentification.Crypto cryptoLocation;

    public enum Crypto {
        Left,
        Center,
        Right,
        Unknown
    }

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    double heading = 0;
    int rotations = 0;
    /* local OpMode members. */
    HardwareMap hwMap           =  null;

    /* Constructor */
    public HardwareTOBOR(){

    }

    /* Initialize standard Hardware interfaces */
    public void initRobo(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        FRMotor = hwMap.dcMotor.get("FRight");
        FLMotor = hwMap.dcMotor.get("FLeft");
        BRMotor = hwMap.dcMotor.get("BRight");
        BLMotor = hwMap.dcMotor.get("BLeft");
        rightTrack = hwMap.dcMotor.get("RTrack");
        leftTrack = hwMap.dcMotor.get("LTrack");
        RPlate = hwMap.servo.get("RPlate");
        LPlate = hwMap.servo.get("LPlate");


        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        rightTrack.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        FRMotor.setPower(0);
        FLMotor.setPower(0);
        BRMotor.setPower(0);
        BLMotor.setPower(0);
        rightTrack.setPower(0);
        leftTrack.setPower(0);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Define and initialize ALL installed servos.

    }

    public void initVuforia()
    {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AcZlc3n/////AAAAGWPeDCNLuk38gPuwF9cpyK2BYbGciGSeJy9AkSXPprQUEtg/VxgqB6j9WJuQvGo4pq+h4gwPSd134WD707FXnbuJjqdqkh5/92mATPs96WQ2RVoaU8QLbsJonufIl2T6qqqT83aOJHbz34mGJszad+Mw7VAWM11av5ltOoq8/rSKbmSFxAVi3d7oiT3saE0XBx4svhpGLwauy6Y0L7X0fC7FwHKCnw/RPL4V+Q8v2rtCTOwvjfnjxmRMind01HSWcxd9ppBwzvHVCPhePccnyWVv5jNiYXia9r4FlrJpAPgZ1GsCfdbt6AoT6Oh2Hnx267J+MHUnLi/C+0brvnQfcDregLBfnZApfd2c1WDiXJp/";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        relicTrackables.activate();
        cryptoLocation = TOBORVuMarkIdentification.Crypto.Unknown;
    }

    public void strafeLeft(double power)
    {
        BLMotor.setPower(power);
        BRMotor.setPower(-power);
        FRMotor.setPower(power);
        FLMotor.setPower(-power);
    }

    public void strafeLeftAuto (double power)
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double adjustment = angles.firstAngle/180*0.3;
        BLMotor.setPower(power + adjustment);
        BRMotor.setPower(-power - adjustment);
        FRMotor.setPower(power - adjustment);
        FLMotor.setPower(-power + adjustment);
    }
    public void strafeRight(double power)
    {
        BLMotor.setPower(-power);
        BRMotor.setPower(power);
        FRMotor.setPower(-power);
        FLMotor.setPower(power);
    }

    public void strafeRightAuto(double power)
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double adjustment = angles.firstAngle/180*0.29;
        BLMotor.setPower(-power + adjustment);
        BRMotor.setPower(power - adjustment);
        FRMotor.setPower(-power - adjustment);
        FLMotor.setPower(power + adjustment);
    }

    public TOBORVuMarkIdentification.Crypto seekImage()
    {
        if(vuMark == RelicRecoveryVuMark.LEFT){
            cryptoLocation = TOBORVuMarkIdentification.Crypto.Left;

        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            cryptoLocation = TOBORVuMarkIdentification.Crypto.Center;

        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT){
            cryptoLocation = TOBORVuMarkIdentification.Crypto.Right;

        }

        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {


        }
        return cryptoLocation;
    }
 }

