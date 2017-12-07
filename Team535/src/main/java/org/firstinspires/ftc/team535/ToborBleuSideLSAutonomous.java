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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.team535.ToborBleuSideLSAutonomous.Auto.center;
import static org.firstinspires.ftc.team535.ToborBleuSideLSAutonomous.Auto.left;
import static org.firstinspires.ftc.team535.ToborBleuSideLSAutonomous.Auto.offStone;
import static org.firstinspires.ftc.team535.ToborBleuSideLSAutonomous.Auto.readImage;
import static org.firstinspires.ftc.team535.ToborBleuSideLSAutonomous.Auto.right;
import java.lang.Math;
/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TOBORBleuSideLSAutonomous", group="Autonomous")
//@Disabled
public class ToborBleuSideLSAutonomous extends OpMode
{
    HardwareTOBOR robo = new HardwareTOBOR();
public enum Auto{readImage, offStone, left, center, right, dispense, end }
    public int location;
    Auto blueSide;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    double heading = 0;
    int rotations = 0;
    @Override
    public void init() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Status", "Initialized");
    robo.initRobo(hardwareMap);
        robo.initVuforia();
        blueSide = readImage;
        location = 0;
    }


    @Override
    public void init_loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("1", "Integrated Heading: " + getIntegratedHeading());
        telemetry.addData("2", "heading: " + angles.firstAngle);
        robo.seekImage();
        if (robo.cryptoLocation == TOBORVuMarkIdentification.Crypto.Left)
        {
            telemetry.addData("Vumark Left","Acquired");
            location = 1;
        }
        else if (robo.cryptoLocation == TOBORVuMarkIdentification.Crypto.Center)
        {
            telemetry.addData("Vumark Center","Acquired");
            location = 2;
        }
        else if (robo.cryptoLocation == TOBORVuMarkIdentification.Crypto.Right)
        {
            telemetry.addData("Vumark Right","Acquired");
            location = 3;
        }
        else
        {
            telemetry.addData("Vumark", "Unknown");
            location = 0;
        }
    }

    @Override
    public void loop() {
        robo.BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robo.BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robo.FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robo.FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        switch (blueSide) {
            case readImage:
                if (location != 0) {
                   blueSide = offStone;
                }
                break;
            case offStone:
                robo.FRMotor.setTargetPosition(2272);
                if (robo.FRMotor.getCurrentPosition()!=2272){
                    robo.strafeRight(.5);

                }
                break;
            case left:

                break;
            case center:

                break;
            case right:

                break;
                }
    }
    @Override
    public void stop() {
    }

}
