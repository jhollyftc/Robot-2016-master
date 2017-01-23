package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.DistanceSensor;



/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class HardwareDM
{
    /* Public OpMode members. */

    /* Drive train motors */
    public DcMotor  lfDrive = null;
    public DcMotor  lrDrive = null;
    public DcMotor  rfDrive = null;
    public DcMotor  rrDrive = null;

    /* Shooter motors */
    public DcMotor lShoot = null;
    public DcMotor rShoot = null;

    /*  Intake motor */
    public DcMotor intake = null;

    /* Shooter feed cam */
    public CRServo fire = null;

    public Servo beacon = null;
    public Servo pivot = null;

    /* Ultrasonic Sensor */
    public I2cDevice rangeUSS;
    public I2cDeviceSynch rangeUSSreader;

    /* ODS Sensor */
    public I2cDevice rangeODS;
    public I2cDeviceSynch rangeODSreader;

    // The IMU sensor object
    BNO055IMU imu;

    /* Adafruit RGB Sensor */
    ColorSensor sensorRGB;

    DeviceInterfaceModule cdim;

    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int LED_CHANNEL = 0;


    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareDM() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Modern Robotics Ultrasonic Sensor
        rangeUSS = hwMap.i2cDevice.get("rangeUSS");
        rangeUSSreader = new I2cDeviceSynchImpl(rangeUSS, I2cAddr.create8bit(0x28), false);
        rangeUSSreader.engage();

        // Define and Initialize Motors
        lfDrive   = hwMap.dcMotor.get("lf motor");
        lrDrive   = hwMap.dcMotor.get("lr motor");
        rfDrive   = hwMap.dcMotor.get("rf motor");
        rrDrive   = hwMap.dcMotor.get("rr motor");
        lShoot    = hwMap.dcMotor.get("l shoot");
        rShoot    = hwMap.dcMotor.get("r shoot");
        intake    = hwMap.dcMotor.get("intake");

        // Define and initialize servos
        fire = hwMap.crservo.get("fire");

        beacon = hwMap.servo.get("beacon");
        pivot = hwMap.servo.get("pivot");

        // Set all motors to zero power
        lfDrive.setPower(0.0);
        lrDrive.setPower(0.0);
        rfDrive.setPower(0.0);
        rrDrive.setPower(0.0);
        lShoot.setPower(0.0);
        rShoot.setPower(0.0);
        fire.setPower(0.0);
        intake.setPower(0.0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rShoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set drive train motor directions
        lfDrive.setDirection(DcMotor.Direction.REVERSE);
        lrDrive.setDirection(DcMotor.Direction.REVERSE);
        rfDrive.setDirection(DcMotor.Direction.FORWARD);
        rrDrive.setDirection(DcMotor.Direction.FORWARD);
        lShoot.setDirection(DcMotor.Direction.REVERSE);
        rShoot.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "gyro");
        imu.initialize(parameters);

        // Retrieve and initialize the Adafruit color sensor
        sensorRGB = hwMap.colorSensor.get("color");

        cdim = hwMap.deviceInterfaceModule.get("dim");
        cdim.setDigitalChannelState(LED_CHANNEL, false); // Turn RGB light off

    }

    public class MRIColorBeacon {

        private byte[] colorBcache;

        private I2cDevice colorB;
        private I2cDeviceSynch colorBreader;

        HardwareMap hwMap = null;


        public MRIColorBeacon() {

        }

        public void init(HardwareMap ahwMap, String cfgName) {
            init(ahwMap, cfgName, 0x4c); //Default I2C address for color beacon is 0x4c
        }

        public void init(HardwareMap ahwMap, String cfgName, int i2cAddr8) {

            hwMap = ahwMap;

            colorB = hwMap.i2cDevice.get(cfgName);

            colorBreader = new I2cDeviceSynchImpl(colorB, I2cAddr.create8bit(i2cAddr8), false);

            colorBreader.engage();
        }

        public void off() {
            colorBreader.write8(4, 0);
        }

        public void red() {
            colorBreader.write8(4, 1);
        }

        public void green() {
            colorBreader.write8(4, 2);
        }

        public void yellow() {
            colorBreader.write8(4, 3);
        }

        public void blue() {
            colorBreader.write8(4, 4);
        }

        public void purple() {
            colorBreader.write8(4, 5);
        }

        public void teal() {
            colorBreader.write8(4, 6);
        }

        public void white() {
            colorBreader.write8(4, 7);
        }

        public void rgb(int red, int green, int blue) {
            colorBreader.write8(4, 8); //Custom Color Mode
            colorBreader.write8(5, red);
            colorBreader.write8(6, green);
            colorBreader.write8(7, blue);
        }

        public int getColorNumber(){
            colorBcache = colorBreader.read(0x04, 4);

            return colorBcache[0];
        }

        public String getColor() {
            colorBcache = colorBreader.read(0x04, 4);

            String returnString = "UNKNOWN";

            switch (colorBcache[0]) {
                case 0:
                    returnString = "OFF";
                    break;
                case 1:
                    returnString = "RED";
                    break;
                case 2:
                    returnString = "GREEN";
                    break;
                case 3:
                    returnString = "YELLOW";
                    break;
                case 4:
                    returnString = "BLUE";
                    break;
                case 5:
                    returnString = "PURPLE";
                    break;
                case 6:
                    returnString = "TEAL";
                    break;
                case 7:
                    returnString = "WHITE";
                    break;
                case 8:
                    returnString = "rgb " + ((int) colorBcache[1] & 0xFF) + " " + ((int) colorBcache[2] & 0xFF) + " " + ((int) colorBcache[3] & 0xFF);
                    break;
                default:
                    returnString = "UNKNOWN";
                    break;
            }

            return returnString;
        }

        public void colorNumber(int number) {
            number = number % 7;

            switch (number) {
                case 0:
                    off();
                    break;
                case 1:
                    red();
                    break;
                case 2:
                    green();
                    break;
                case 3:
                    yellow();
                    break;
                case 4:
                    blue();
                    break;
                case 5:
                    purple();
                    break;
                case 6:
                    teal();
                    break;
                case 7:
                    white();
                    break;
                default:
                    off();
                    break;
            }
        }

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs)  throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
