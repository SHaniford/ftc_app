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

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

import java.util.Timer;


public class HardwareTOBOR
{
    DcMotor FRMotor;
    DcMotor FLMotor;
    DcMotor BRMotor;
    DcMotor BLMotor;
    DcMotor rightTrackUp;
    DcMotor rightTrackDown;
    DcMotor leftTrackUp;
    DcMotor leftTrackDown;
    Servo RPlate;
    Servo LPlate;
    Servo JArm;
    Servo Claw;
    CRServo relicArmTurn;
    CRServo relicArmExtend;
    ColorSensor armSensor;

    
    ModernRoboticsI2cRangeSensor rangeSensor;
    

    VuforiaLocalizer vuforia;
    
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    private VuforiaTrackableDefaultListener relicTemplateListener;
    public ElapsedTime runtime = new ElapsedTime();
    double JArmUpVal = .3067;
    public enum Crypto {
        Left,
        Center,
        Right,
        Unknown
    }
    public enum armPos{
        Down,
        Up,
        Back
    }
    public enum color{
        Red,
        Blue
    }
    public enum direction
    {
        Right,
        Left,
        Unknown
    }
    float hsvValues[] = {0F,0F,0F};
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
        rightTrackUp = hwMap.dcMotor.get("RTrackUp");
        rightTrackDown = hwMap.dcMotor.get("RTrackDown");
        leftTrackUp = hwMap.dcMotor.get("LTrackUp");
        leftTrackDown = hwMap.dcMotor.get("LTrackDown");
        RPlate = hwMap.servo.get("RPlate");
        LPlate = hwMap.servo.get("LPlate");
        relicArmTurn = hwMap.crservo.get("relicArmTurn");
        relicArmExtend = hwMap.crservo.get("relicArmExtend");
        rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        JArm = hwMap.servo.get("Arm");
        armSensor = hwMap.colorSensor.get("armSensor");
        Claw = hwMap.servo.get("claw");


        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        rightTrackUp.setDirection(DcMotor.Direction.REVERSE);
        rightTrackDown.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        FRMotor.setPower(0);
        FLMotor.setPower(0);
        BRMotor.setPower(0);
        BLMotor.setPower(0);
        rightTrackUp.setPower(0);
        rightTrackDown.setPower(0);
        leftTrackUp.setPower(0);
        leftTrackDown.setPower(0);
        RPlate.setPosition(.81);
        LPlate.setPosition(.27);
        armSensor.enableLed(false);

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
        parameters.useExtendedTracking = false;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");
        relicTemplateListener = (VuforiaTrackableDefaultListener) relicTemplate.getListener();
    }
    public void startVuforia()
    {
        relicTrackables.activate();
    }
    public void stopVuforia()
    {
        relicTrackables.deactivate();
    }
    public RelicRecoveryVuMark readKey()
    {
        return RelicRecoveryVuMark.from(relicTemplate);
    }




  public void arm(armPos armPos)
    {
        if (armPos == HardwareTOBOR.armPos.Up)
        {
            JArm.setPosition(.1927);
            armSensor.enableLed(false);
        }
        else if (armPos == HardwareTOBOR.armPos.Down)
        {
            JArm.setPosition(.8744);
            armSensor.enableLed(true);
        }
        else if (armPos == HardwareTOBOR.armPos.Back)
        {
            JArm.setPosition(0);
            armSensor.enableLed(false);
        }
    }
    public direction knockJewel (color targetColor)
    {
        if (targetColor == color.Red) {
            Color.RGBToHSV(armSensor.red() * 8, armSensor.green() * 8, armSensor.blue() * 8, hsvValues);
            if ((hsvValues[0] >= 335 || hsvValues[0] <= 25) && hsvValues[1] >= 0.5) {
                return direction.Left;
            } else if (hsvValues[0] >= 205 && hsvValues[0] <= 270 && hsvValues[1] >= 0.5) {
                return direction.Right;
            }
            else if (runtime.time() >= 2)
            {
                JArm.setPosition(JArm.getPosition()-0.002);
                return direction.Unknown;
            }
            else
            {
                return direction.Unknown;
            }

        }
        else if (targetColor == color.Blue)
        {
            Color.RGBToHSV(armSensor.red() * 8, armSensor.green() * 8, armSensor.blue() * 8, hsvValues);
            if ((hsvValues[0] >= 237 || hsvValues[0] <= 18)) {
                return direction.Right;
            } else if (hsvValues[0] >= 152 && hsvValues[0] <= 191) {
                return direction.Left;
            }
            else
            {
                return direction.Unknown;
            }

        }
        else
        {
            return direction.Unknown;
        }



    }


    public void strafeLeft(double power)
    {
        BLMotor.setPower(power);
        BRMotor.setPower(-power);
        FRMotor.setPower(power);
        FLMotor.setPower(-power);
    }

    public double strafeLeftAuto (double power)
    {
        double distance = (16-rangeSensor.getDistance(DistanceUnit.INCH))/100;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double adjustment = angles.firstAngle/40;
        BLMotor.setPower(power - adjustment + distance);
        BRMotor.setPower(-power + adjustment + distance );
        FRMotor.setPower(power + adjustment + distance);
        FLMotor.setPower(-power - adjustment + distance);
        return angles.firstAngle;

    }
    public void strafeRight(double power)
    {
        BLMotor.setPower(-power);
        BRMotor.setPower(power);
        FRMotor.setPower(-power);
        FLMotor.setPower(power);
    }

    public double strafeRightAuto(double power)
    {
        double distance = (16-rangeSensor.getDistance(DistanceUnit.INCH))/100;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double adjustment = angles.firstAngle/40;
        BLMotor.setPower(-power - adjustment +distance);
        BRMotor.setPower(power + adjustment + distance);
        FRMotor.setPower(-power + adjustment + distance);
        FLMotor.setPower(power - adjustment + distance);
        return angles.firstAngle;

    }
        public double DriveForwardAuto(double power, int offset)
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double adjustment = (angles.firstAngle-offset)/40;
        BLMotor.setPower(power - adjustment);
        BRMotor.setPower(power + adjustment);
        FRMotor.setPower(power + adjustment);
        FLMotor.setPower(power - adjustment);
        return angles.firstAngle;

    }
 }

