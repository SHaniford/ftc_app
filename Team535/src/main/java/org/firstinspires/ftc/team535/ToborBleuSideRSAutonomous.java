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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.VuMarkTarget;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static org.firstinspires.ftc.team535.ToborBleuSideRSAutonomous.Auto.DFWD;
import static org.firstinspires.ftc.team535.ToborBleuSideRSAutonomous.Auto.center;
import static org.firstinspires.ftc.team535.ToborBleuSideRSAutonomous.Auto.dispense;
import static org.firstinspires.ftc.team535.ToborBleuSideRSAutonomous.Auto.endAll;
import static org.firstinspires.ftc.team535.ToborBleuSideRSAutonomous.Auto.forward;
import static org.firstinspires.ftc.team535.ToborBleuSideRSAutonomous.Auto.jolt;
import static org.firstinspires.ftc.team535.ToborBleuSideRSAutonomous.Auto.left;
import static org.firstinspires.ftc.team535.ToborBleuSideRSAutonomous.Auto.offStone;
import static org.firstinspires.ftc.team535.ToborBleuSideRSAutonomous.Auto.right;

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

@Autonomous(name="TOBORBleuSideRSAutonomous", group="Autonomous")
@Disabled
public class ToborBleuSideRSAutonomous extends OpMode
{
    public RelicRecoveryVuMark roboLocation;
    

    HardwareTOBOR robo = new HardwareTOBOR();
public enum Auto{readImage, readJewel, knockJewel, replace, offStone, DFWD, right, left, center, forward, dispense, jolt, endAll}
Auto blueSide;
    HardwareTOBOR.direction dir;
    double heading;
    double TPI = 43;
    private RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robo.initRobo(hardwareMap);
        robo.initVuforia();
        robo.startVuforia();
        robo.BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robo.BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blueSide = Auto.readImage;

    }


    @Override
    public void init_loop() {
        if (robo.readKey() != RelicRecoveryVuMark.UNKNOWN)
        {
            vuMark = robo.readKey();
            telemetry.addData("Vumark Acquired", vuMark);
        }

    }
    @Override
    public void start()
    {
        robo.stopVuforia();
        robo.BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robo.BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robo.FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robo.FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    @Override
    public void loop() {
        telemetry.addData("Current Pos", robo.BRMotor.getCurrentPosition());
        telemetry.addData("Current State", blueSide);
        switch (blueSide) {
            case readImage:
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    blueSide = Auto.readJewel;
                }
                break;
            case readJewel:
                robo.arm(HardwareTOBOR.armPos.Down);
                dir = robo.knockJewel(HardwareTOBOR.color.Blue);
                telemetry.addData("Direction", robo.knockJewel(HardwareTOBOR.color.Blue));
                if (dir != HardwareTOBOR.direction.Unknown) {
                    blueSide = Auto.knockJewel;
                }
                break;
            case knockJewel:
                robo.angles = robo.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if (dir == HardwareTOBOR.direction.Left) {
                    robo.BRMotor.setPower(0.3);
                    robo.FRMotor.setPower(0.3);
                    robo.BLMotor.setPower(-0.3);
                    robo.FLMotor.setPower(-0.3);
                    if (robo.angles.firstAngle >= 15) {
                        blueSide = Auto.replace;
                    } else if (dir == HardwareTOBOR.direction.Right) {
                        robo.BRMotor.setPower(-0.3);
                        robo.FRMotor.setPower(-0.3);
                        robo.BLMotor.setPower(0.3);
                        robo.FLMotor.setPower(0.3);
                        if (robo.angles.firstAngle <= -15) {
                            blueSide = Auto.replace;
                        }
                    } else {
                        blueSide = Auto.readJewel;
                    }
                }
                break;
            case replace:
                if (dir == HardwareTOBOR.direction.Left) {
                    robo.BRMotor.setPower(-0.3);
                    robo.FRMotor.setPower(-0.3);
                    robo.BLMotor.setPower(0.3);
                    robo.FLMotor.setPower(0.3);
                    if (robo.angles.firstAngle <= 91 && robo.angles.firstAngle >= 89) {
                        blueSide = Auto.offStone;
                    }
                } else if (dir == HardwareTOBOR.direction.Right) {
                    robo.BRMotor.setPower(0.3);
                    robo.FRMotor.setPower(0.3);
                    robo.BLMotor.setPower(-0.3);
                    robo.FLMotor.setPower(-0.3);
                    if (robo.angles.firstAngle <= 91 && robo.angles.firstAngle >= 89 ) {
                        blueSide = Auto.DFWD;
                    }
                }
                break;
            case DFWD:
                robo.DriveForwardAuto(-0.2,0);
if(((19*TPI)+robo.BRMotor.getCurrentPosition()< (0.5*TPI)) && (robo.rangeSensor.getDistance(DistanceUnit.INCH) <=10.5) && (robo.rangeSensor.getDistance(DistanceUnit.INCH) >=10))  {
    blueSide = offStone;
            }

                break;
            case offStone:
                heading = robo.strafeLeftAuto(0.35);
                if (((25*TPI)+robo.BRMotor.getCurrentPosition()< (0.5*TPI))&&(robo.rangeSensor.getDistance(DistanceUnit.INCH) >= 10))
                {
                    if (vuMark == RelicRecoveryVuMark.LEFT) {
                        blueSide = left;
                    }
                    if (vuMark == RelicRecoveryVuMark.CENTER) {
                        blueSide = center;
                    }
                    if (vuMark == RelicRecoveryVuMark.RIGHT) {
                        blueSide = forward;
                    }
                }

            case left:
                heading = robo.strafeLeftAuto(0.35);
                if ((46*TPI)-robo.BRMotor.getCurrentPosition()< (0.5*TPI)&&(robo.rangeSensor.getDistance(DistanceUnit.INCH) >= 10))
                {
                    blueSide = forward;
                }
                break;
            case center:
                heading = robo.strafeLeftAuto(0.35);
                telemetry.addData("Distance", Math.abs((36*TPI)+robo.BRMotor.getCurrentPosition()));
                if (((36*TPI)-robo.BRMotor.getCurrentPosition()< (0.5*TPI))&&(robo.rangeSensor.getDistance(DistanceUnit.INCH) >= 10))
                {
                    blueSide = forward;
                }
                break;
            case forward:
heading = robo.DriveForwardAuto(-.35,0);
                if (((2*TPI)-robo.BRMotor.getCurrentPosition()< (0.5*TPI))&&(robo.rangeSensor.getDistance(DistanceUnit.INCH) >= 8))
                {
                    blueSide = dispense;
                }

                break;
            case dispense:
                robo.RPlate.setPosition(.08);
                robo.LPlate.setPosition(1);
                robo.BRMotor.setPower(0);
                robo.FRMotor.setPower(0);
                robo.BLMotor.setPower(0);
                robo.FLMotor.setPower(0);
                if (robo.rangeSensor.getDistance(DistanceUnit.INCH)<= 8)
                {
                    blueSide=jolt;
                }

                break;
            case jolt:
                robo.DriveForwardAuto(0.2,0);
                if (robo.rangeSensor.getDistance(DistanceUnit.INCH) <= 4)
                {
                    blueSide = endAll;
                }
                break;
            case endAll:
                robo.RPlate.setPosition(.81);
                robo.LPlate.setPosition(.27);
                robo.BRMotor.setPower(0);
                robo.FRMotor.setPower(0);
                robo.BLMotor.setPower(0);
                robo.FLMotor.setPower(0);

                break;

        }
    }

        @Override
        public void stop () {
        }
    }

