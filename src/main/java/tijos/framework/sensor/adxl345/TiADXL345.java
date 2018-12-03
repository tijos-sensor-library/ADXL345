package tijos.framework.sensor.adxl345;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.framework.util.LittleBitConverter;

/**
 * TiADXL345 3-Axis, ±2 g/±4 g/±8 g/±16 g Digital Accelerometer driver for TiJOS 
 * 
 */
public class TiADXL345 {

	/* ------- Register names ------- */
	public static final int ADXL345_DEVID = 0x00;
	public static final int ADXL345_RESERVED1 = 0x01;
	public static final int ADXL345_THRESH_TAP = 0x1d;
	public static final int ADXL345_OFSX = 0x1e;
	public static final int ADXL345_OFSY = 0x1f;
	public static final int ADXL345_OFSZ = 0x20;
	public static final int ADXL345_DUR = 0x21;
	public static final int ADXL345_LATENT = 0x22;
	public static final int ADXL345_WINDOW = 0x23;
	public static final int ADXL345_THRESH_ACT = 0x24;
	public static final int ADXL345_THRESH_INACT = 0x25;
	public static final int ADXL345_TIME_INACT = 0x26;
	public static final int ADXL345_ACT_INACT_CTL = 0x27;
	public static final int ADXL345_THRESH_FF = 0x28;
	public static final int ADXL345_TIME_FF = 0x29;
	public static final int ADXL345_TAP_AXES = 0x2a;
	public static final int ADXL345_ACT_TAP_STATUS = 0x2b;
	public static final int ADXL345_BW_RATE = 0x2c;
	public static final int ADXL345_POWER_CTL = 0x2d;
	public static final int ADXL345_INT_ENABLE = 0x2e;
	public static final int ADXL345_INT_MAP = 0x2f;
	public static final int ADXL345_INT_SOURCE = 0x30;
	public static final int ADXL345_DATA_FORMAT = 0x31;
	public static final int ADXL345_DATAX0 = 0x32;
	public static final int ADXL345_DATAX1 = 0x33;
	public static final int ADXL345_DATAY0 = 0x34;
	public static final int ADXL345_DATAY1 = 0x35;
	public static final int ADXL345_DATAZ0 = 0x36;
	public static final int ADXL345_DATAZ1 = 0x37;
	public static final int ADXL345_FIFO_CTL = 0x38;
	public static final int ADXL345_FIFO_STATUS = 0x39;

	public static final int ADXL345_BW_1600 = 0xF; // 1111
	public static final int ADXL345_BW_800 = 0xE; // 1110
	public static final int ADXL345_BW_400 = 0xD; // 1101
	public static final int ADXL345_BW_200 = 0xC; // 1100
	public static final int ADXL345_BW_100 = 0xB; // 1011
	public static final int ADXL345_BW_50 = 0xA; // 1010
	public static final int ADXL345_BW_25 = 0x9; // 1001
	public static final int ADXL345_BW_12 = 0x8; // 1000
	public static final int ADXL345_BW_6 = 0x7; // 0111
	public static final int ADXL345_BW_3 = 0x6; // 0110

	/*
	 * Interrupt PINs INT1: 0 INT2: 1
	 */
	public static final int ADXL345_INT1_PIN = 0x00;
	public static final int ADXL345_INT2_PIN = 0x01;

	/* Interrupt bit position */
	public static final int ADXL345_INT_DATA_READY_BIT = 0x07;
	public static final int ADXL345_INT_SINGLE_TAP_BIT = 0x06;
	public static final int ADXL345_INT_DOUBLE_TAP_BIT = 0x05;
	public static final int ADXL345_INT_ACTIVITY_BIT = 0x04;
	public static final int ADXL345_INT_INACTIVITY_BIT = 0x03;
	public static final int ADXL345_INT_FREE_FALL_BIT = 0x02;
	public static final int ADXL345_INT_WATERMARK_BIT = 0x01;
	public static final int ADXL345_INT_OVERRUNY_BIT = 0x00;

	public static final int ADXL345_DATA_READY = 0x07;
	public static final int ADXL345_SINGLE_TAP = 0x06;
	public static final int ADXL345_DOUBLE_TAP = 0x05;
	public static final int ADXL345_ACTIVITY = 0x04;
	public static final int ADXL345_INACTIVITY = 0x03;
	public static final int ADXL345_FREE_FALL = 0x02;
	public static final int ADXL345_WATERMARK = 0x01;
	public static final int ADXL345_OVERRUNY = 0x00;

	public static final int ADXL345_OK = 1; // no error
	public static final int ADXL345_ERROR = 0; // indicates error is predent

	public static final int ADXL345_NO_ERROR = 0; // initial state
	public static final int ADXL345_READ_ERROR = 1; // problem reading accel
	public static final int ADXL345_BAD_ARG = 2; // bad method argument

	private static final int ADXL345_DEVICE = 0x53; // ADXL345 device address
	private static final int ADXL345_TO_READ = 6; // num of bytes we are going to read each time (two bytes for each
													// axis)

	private double[] _gains = new double[3];

	private byte[] _buff = new byte[8];
	private int[] _xyzi = new int[3];
	private double[] _xyzd = new double[3];

	private TiI2CMaster _i2c = null;

	/**
	 * Initialize with I2C object
	 * @param i2c
	 */
	public TiADXL345(TiI2CMaster i2c) {
		this._i2c = i2c;
		this._gains[0] = 0.0039;
		this._gains[1] = 0.0039;
		this._gains[2] = 0.0039;
	}

	/**
	 * // Turning on the ADXL345
	 * @throws IOException
	 */
	public void powerOn() throws IOException {
		
		this.writeTo(ADXL345_POWER_CTL, 0);
		this.writeTo(ADXL345_POWER_CTL, 16);
		this.writeTo(ADXL345_POWER_CTL, 8);
	}

	/**
	 * Read device ID 
	 * @return device id
	 * @throws IOException
	 */
	public int readID() throws IOException {
		byte[] buff = this.readFrom(ADXL345_DEVID, 1);
		return buff[0] & 0xff;
	}

	/**
	 * read the acceleration data
	 * @return acceleration data
	 * @throws IOException
	 */
	public int[] readXYZ() throws IOException {
		byte[] buf = this.readFrom(ADXL345_DATAX0, ADXL345_TO_READ); 
		this._xyzi[0] = LittleBitConverter.ToInt16(buf, 0);
		this._xyzi[1] = LittleBitConverter.ToInt16(buf, 2);
		this._xyzi[2] = LittleBitConverter.ToInt16(buf, 4);
		return this._xyzi;
	}

	/**
	 * read acceleration of gravity
	 * @return acceleration of gravity
	 * @throws IOException
	 */
	public double[] readGxyz() throws IOException {
		int i;
		int[] xyz;
		xyz = this.readXYZ();
		for (i = 0; i < 3; i++) {
			this._xyzd[i] = xyz[i] * this._gains[i];
		}
		return this._xyzd;
	}


	/**
	 * Gets the range setting and return it into rangeSetting,it can be 2, 4, 8 or 16
	 * @return 
	 * @throws IOException
	 */
	public int getRangeSetting() throws IOException {
		byte[] buff = this.readFrom(ADXL345_DATA_FORMAT, 1);
		return buff[0] & 3;
	}

	/**
	 * Sets the range setting, possible values are: 2, 4, 8, 16
	 * @param val
	 * @throws IOException
	 */
	public void setRangeSetting(int val) throws IOException {
		int s;
		int b;

		switch (val) {
		case 2:
			s = 0;
			break;
		case 4:
			s = 1;
			break;
		case 8:
			s = 2;
			break;
		case 16:
			s = 3;
			break;
		default:
			s = 0;
		}
		byte[] buff = this.readFrom(ADXL345_DATA_FORMAT, 1);
		b = buff[0];
		s |= (b & 0xec);
		this.writeTo(ADXL345_DATA_FORMAT, s);
	}

	/**
	 * gets the state of the SELF_TEST bit
	 * @return
	 * @throws IOException
	 */
	public int getSelfTestBit() throws IOException {
		return this.getRegisterBit(ADXL345_DATA_FORMAT, 7);
	}

	/**
	 * Sets the SELF-TEST bit if set to 1 it applies a self-test force to the sensor causing a shift in the	 output data
	 * if set to 0 it disables the self-test force
	 * @param selfTestBit
	 * @throws IOException
	 */
	public void setSelfTestBit(int selfTestBit) throws IOException {
		this.setRegisterBit(ADXL345_DATA_FORMAT, 7, selfTestBit);
	}

	/**
	 * Gets the state of the SPI bit
	 * @return
	 * @throws IOException
	 */
	public int getSpiBit() throws IOException {
		return this.getRegisterBit(ADXL345_DATA_FORMAT, 6);
	}

	/**
	 * Sets the SPI bit
	 * if set to 1 it sets the device to 3-wire mode
	 *  if set to 0 it sets the device to 4-wire SPI mode
	 * @param spiBit
	 * @throws IOException
	 */
	public void setSpiBit(int spiBit) throws IOException {
		this.setRegisterBit(ADXL345_DATA_FORMAT, 6, spiBit);
	}

	/**
	 * Gets the state of the INT_INVERT bit
	 * @return
	 * @throws IOException
	 */
	public int getInterruptLevelBit() throws IOException {
		return (this.getRegisterBit(ADXL345_DATA_FORMAT, 5) > 0) ? 0 : 1;
	}

	/**
	 * Sets the INT_INVERT bit
	 * if set to 1 sets the interrupts to active high
	 * if set to 0 sets the interrupts to active low
	 * @param interruptLevelBit
	 * @throws IOException
	 */
	public void setInterruptLevelBit(int interruptLevelBit) throws IOException {
		this.setRegisterBit(ADXL345_DATA_FORMAT, 5, (interruptLevelBit > 0) ? 0 : 1);
	}

	/**
	 *  Gets the state of the FULL_RES bit
	 * @return
	 * @throws IOException
	 */
	public int getFullResBit() throws IOException {
		return this.getRegisterBit(ADXL345_DATA_FORMAT, 3);
	}

	/** Sets the FULL_RES bit
	* if set to 1, the device is in full resolution mode, where the output
	* resolution increases with the
	* g range set by the range bits to maintain a 4mg/LSB scal factor
	* if set to 0, the device is in 10-bit mode, and the range buts determine the
	* maximum g range
	* and scale factor
	*/
	public void setFullResBit(int fullResBit) throws IOException {
		this.setRegisterBit(ADXL345_DATA_FORMAT, 3, fullResBit);
	}

	/**
	 * Gets the state of the justify bit
	 * @return
	 * @throws IOException
	 */
	public int getJustifyBit() throws IOException {
		return this.getRegisterBit(ADXL345_DATA_FORMAT, 2);
	}

	/**
	 * Sets the JUSTIFY bit
	 * if sets to 1 selects the left justified mode
	 * if sets to 0 selects right justified mode with sign extension
	 * @param justifyBit
	 * @throws IOException
	 */
	public void setJustifyBit(int justifyBit) throws IOException {
		this.setRegisterBit(ADXL345_DATA_FORMAT, 2, justifyBit);
	}

	/** Sets the THRESH_TAP byte value
	* it should be between 0 and 255
	* the scale factor is 62.5 mg/LSB
	* A value of 0 may result in undesirable behavior
	*/
	public void setTapThreshold(int tapThreshold) throws IOException {
		this.writeTo(ADXL345_THRESH_TAP, tapThreshold);
	}

	/** Gets the THRESH_TAP byte value
	* return value is comprised between 0 and 255
	* the scale factor is 62.5 mg/LSB
	*/
	public int getTapThreshold() throws IOException {
		byte[] buff = this.readFrom(ADXL345_THRESH_TAP, 1);
		return buff[0] & 0xff;
	}

	/**
	 *  set the gain for each axis in Gs / count
	 * @param gains
	 */
	public void setAxisGains(double[] gains) {
		int i;
		for (i = 0; i < 3; i++) {
			this._gains[i] = gains[i];
		}
	}

	/**
	 *  get the gain for each axis in Gs / count
	 * @return
	 */
	public double[] getAxisGains() {
		return this._gains;
	}

	/**
	* Sets the OFSX, OFSY and OFSZ bytes
	* OFSX, OFSY and OFSZ are user offset adjustments in twos complement format
	* with
	* a scale factor of 15.6mg/LSB
	* OFSX, OFSY and OFSZ should be comprised between
	 */
	public void setAxisOffset(int x, int y, int z) throws IOException {
		this.writeTo(ADXL345_OFSX, x);
		this.writeTo(ADXL345_OFSY, y);
		this.writeTo(ADXL345_OFSZ, z);
	}

	/**
	 *  Gets the OFSX, OFSY and OFSZ bytes
	 * @return
	 * @throws IOException
	 */
	public int[] getAxisOffset() throws IOException {
		byte[] buff = this.readFrom(ADXL345_OFSX, 1);
		this._xyzi[0] = buff[0] & 0xff;
		buff = this.readFrom(ADXL345_OFSY, 1);
		this._xyzi[1] = buff[0] & 0xff;
		buff = this.readFrom(ADXL345_OFSZ, 1);
		this._xyzi[2] = buff[0] & 0xff;
		return this._xyzi;
	}

	/** Sets the DUR byte
	 * The DUR byte contains an unsigned time value representing the maximum time
	 * that an event must be above THRESH_TAP threshold to qualify as a tap event
	 * The scale factor is 625µs/LSB
	 * A value of 0 disables the tap/double tap funcitons. Max value is 255.
	 */
	public void setTapDuration(int tapDuration) throws IOException {
		this.writeTo(ADXL345_DUR, tapDuration);
	}

	/**
	 * Gets the DUR byte
	 * @return
	 * @throws IOException
	 */
	public int getTapDuration() throws IOException {
		byte[] buff = this.readFrom(ADXL345_DUR, 1);
		return buff[0] & 0xff;
	}

	/**
	 * Sets the latency (latent register) which contains an unsigned time value
	 * representing the wait time from the detection of a tap event to the start
	 * of the time window, during which a possible second tap can be detected.
	 * The scale factor is 1.25ms/LSB. A value of 0 disables the double tap
	 * function.
	 * It accepts a maximum value of 255.
	 */
	public void setDoubleTapLatency(int doubleTapLatency) throws IOException {
		this.writeTo(ADXL345_LATENT, doubleTapLatency);
	}

	/**
	* Gets the Latent value
	*/
	public int getDoubleTapLatency() throws IOException {
		byte[] buff = this.readFrom(ADXL345_LATENT, 1);
		return buff[0] & 0xff;
	}

	/** 
	 * Sets the Window register, which contains an unsigned time value representing
	* the amount of time after the expiration of the latency time (Latent register)
	* during which a second valud tap can begin. The scale factor is 1.25ms/LSB. A
	* value of 0 disables the double tap function. The maximum value is 255.
	*/
	public void setDoubleTapWindow(int doubleTapWindow) throws IOException {
		this.writeTo(ADXL345_WINDOW, doubleTapWindow);
	}

	/**
	 * Gets the Window register
	 * @return
	 * @throws IOException
	 */
	public int getDoubleTapWindow() throws IOException {
		byte[] buff = this.readFrom(ADXL345_WINDOW, 1);
		return buff[0] & 0xff;
	}

	/**
	* Sets the THRESH_ACT byte which holds the threshold value for detecting
	* activity.
	* The data format is unsigned, so the magnitude of the activity event is
	* compared
	* with the value is compared with the value in the THRESH_ACT register. The
	* scale
	* factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
	* activity interrupt is enabled. The maximum value is 255.
	*/
	public void setActivityThreshold(int activityThreshold) throws IOException {
		this.writeTo(ADXL345_THRESH_ACT, activityThreshold);
	}

	/**
	 * Gets the THRESH_ACT byte
	 * @return
	 * @throws IOException
	 */
	public int getActivityThreshold() throws IOException {
		byte[] buff = this.readFrom(ADXL345_THRESH_ACT, 1);
		return buff[0] & 0xff;
	}

	
	/**
	 * Sets the THRESH_INACT byte which holds the threshold value for detecting
	 * inactivity.
	 * The data format is unsigned, so the magnitude of the inactivity event is
	 * compared with the value is compared with the value in the THRESH_INACT register. The
	 * scale factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
	 * inactivity interrupt is enabled. The maximum value is 255.
	 */
	public void setInactivityThreshold(int inactivityThreshold) throws IOException {
		this.writeTo(ADXL345_THRESH_INACT, inactivityThreshold);
	}

	/**
	 * Gets the THRESH_INACT byte
	 * @return
	 * @throws IOException
	 */
	public int getInactivityThreshold() throws IOException {
		byte[] buff = this.readFrom(ADXL345_THRESH_INACT, 1);
		return buff[0] & 0xff;
	}

	/**
	 *  Sets the TIME_INACT register, which contains an unsigned time value
	* representing the amount of time that acceleration must be less thant the value in the
	* THRESH_INACTregister for inactivity to be declared. The scale factor is 1sec/LSB. The
	* value must be between 0 and 255.
	*/
	public void setTimeInactivity(int timeInactivity) throws IOException {
		this.writeTo(ADXL345_TIME_INACT, timeInactivity);
	}

	/**
	 * Gets the TIME_INACT register
	 * @return
	 * @throws IOException
	 */
	public int getTimeInactivity() throws IOException {
		byte[] buff = this.readFrom(ADXL345_TIME_INACT, 1);
		return buff[0] & 0xff;
	}

	/**
	 *  Sets the THRESH_FF register which holds the threshold value, in an unsigned
	 * format, for free-fall detection. The root-sum-square (RSS) value of all axes is
	 * calculated and compared whith the value in THRESH_FF to determine if a free-fall event
	 * occured. The scale factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior
	 * if the free-fall interrupt is enabled. The maximum value is 255.
	 */
	public void setFreeFallThreshold(int freeFallThreshold) throws IOException {
		this.writeTo(ADXL345_THRESH_FF, freeFallThreshold);
	}

	/**
	 * Gets the THRESH_FF register.
	 * @return
	 * @throws IOException
	 */
	public int getFreeFallThreshold() throws IOException {
		byte[] buff = this.readFrom(ADXL345_THRESH_FF, 1);
		return buff[0] & 0xff;
	}

	/**
	 * Sets the TIME_FF register, which holds an unsigned time value representing
	*  the minimum time that the RSS value of all axes must be less than THRESH_FF to generate a
	*  free-fall interrupt. The scale factor is 5ms/LSB. A value of 0 may result in
	*  undesirable behavior if the free-fall interrupt is enabled. The maximum value is 255.
	 */
	public void setFreeFallDuration(int freeFallDuration) throws IOException {
		this.writeTo(ADXL345_TIME_FF, freeFallDuration);
	}

	/**
	 *  Gets the TIME_FF register.
	 * @return
	 * @throws IOException
	 */
	public int getFreeFallDuration() throws IOException {
		byte[] buff = this.readFrom(ADXL345_TIME_FF, 1);
		return buff[0] & 0xff;
	}

	/**
	 * Axis enable control for activity and inactivity detection 
	 * @return
	 * @throws IOException
	 */
	public boolean isActivityXEnabled() throws IOException {
		return this.getRegisterBit(ADXL345_ACT_INACT_CTL, 6) > 0;
	}

	public boolean isActivityYEnabled() throws IOException {
		return this.getRegisterBit(ADXL345_ACT_INACT_CTL, 5) > 0;
	}

	public boolean isActivityZEnabled() throws IOException {
		return this.getRegisterBit(ADXL345_ACT_INACT_CTL, 4) > 0;
	}

	public boolean isInactivityXEnabled() throws IOException {
		return this.getRegisterBit(ADXL345_ACT_INACT_CTL, 2) > 0;
	}

	public boolean isInactivityYEnabled() throws IOException {
		return this.getRegisterBit(ADXL345_ACT_INACT_CTL, 1) > 0;
	}

	public boolean isInactivityZEnabled() throws IOException {
		return this.getRegisterBit(ADXL345_ACT_INACT_CTL, 0) > 0;
	}

	public void setActivityX(boolean state) throws IOException {
		this.setRegisterBit(ADXL345_ACT_INACT_CTL, 6, state ? 1 : 0);
	}

	public void setActivityY(boolean state) throws IOException {
		this.setRegisterBit(ADXL345_ACT_INACT_CTL, 5, state ? 1 : 0);
	}

	public void setActivityZ(boolean state) throws IOException {
		this.setRegisterBit(ADXL345_ACT_INACT_CTL, 4, state ? 1 : 0);
	}

	public void setInactivityX(boolean state) throws IOException {
		this.setRegisterBit(ADXL345_ACT_INACT_CTL, 2, state ? 1 : 0);
	}

	public void setInactivityY(boolean state) throws IOException {
		this.setRegisterBit(ADXL345_ACT_INACT_CTL, 1, state ? 1 : 0);
	}

	public void setInactivityZ(boolean state) throws IOException {
		this.setRegisterBit(ADXL345_ACT_INACT_CTL, 0, state ? 1 : 0);
	}

	public boolean isActivityAC() throws IOException {
		return this.getRegisterBit(ADXL345_ACT_INACT_CTL, 7) > 0;
	}

	public boolean isInactivityAC() throws IOException {
		return this.getRegisterBit(ADXL345_ACT_INACT_CTL, 3) > 0;
	}

	public void setActivityAC(boolean state) throws IOException {
		this.setRegisterBit(ADXL345_ACT_INACT_CTL, 7, state ? 1 : 0);
	}

	public void setInactivityAC(boolean state) throws IOException {
		this.setRegisterBit(ADXL345_ACT_INACT_CTL, 3, state ? 1 : 0);
	}

	/**
	 * Axis control for single tap/double tap 
	 * @return
	 * @throws IOException
	 */
	public boolean getSuppressBit() throws IOException {
		return this.getRegisterBit(ADXL345_TAP_AXES, 3) > 0;
	}

	public void setSuppressBit(boolean state) throws IOException {
		this.setRegisterBit(ADXL345_TAP_AXES, 3, state ? 1 : 0);
	}

	public boolean isTapDetectionOnX() throws IOException {
		return this.getRegisterBit(ADXL345_TAP_AXES, 2) > 0;
	}

	public void setTapDetectionOnX(boolean state) throws IOException {
		this.setRegisterBit(ADXL345_TAP_AXES, 2, state ? 1 : 0);
	}

	public boolean isTapDetectionOnY() throws IOException {
		return this.getRegisterBit(ADXL345_TAP_AXES, 1) > 0;
	}

	public void setTapDetectionOnY(boolean state) throws IOException {
		this.setRegisterBit(ADXL345_TAP_AXES, 1, state ? 1 : 0);
	}

	public boolean isTapDetectionOnZ() throws IOException {
		return this.getRegisterBit(ADXL345_TAP_AXES, 0) > 0;
	}

	public void setTapDetectionOnZ(boolean state) throws IOException {
		this.setRegisterBit(ADXL345_TAP_AXES, 0, state ? 1 : 0);
	}

	/**
	 * Source of single tap/double tap 
	 * @return
	 * @throws IOException
	 */
	public boolean isActivitySourceOnX() throws IOException {
		return this.getRegisterBit(ADXL345_ACT_TAP_STATUS, 6) > 0;
	}

	public boolean isActivitySourceOnY() throws IOException {
		return this.getRegisterBit(ADXL345_ACT_TAP_STATUS, 5) > 0;
	}

	public boolean isActivitySourceOnZ() throws IOException {
		return this.getRegisterBit(ADXL345_ACT_TAP_STATUS, 4) > 0;
	}

	public boolean isTapSourceOnX() throws IOException {
		return this.getRegisterBit(ADXL345_ACT_TAP_STATUS, 2) > 0;
	}

	public boolean isTapSourceOnY() throws IOException {
		return this.getRegisterBit(ADXL345_ACT_TAP_STATUS, 1) > 0;
	}

	public boolean isTapSourceOnZ() throws IOException {
		return this.getRegisterBit(ADXL345_ACT_TAP_STATUS, 0) > 0;
	}

	public boolean isAsleep() throws IOException {
		return this.getRegisterBit(ADXL345_ACT_TAP_STATUS, 3) > 0;
	}

	/**
	 * Data rate and power mode control 
	 */
	public boolean isLowPower() throws IOException {
		return this.getRegisterBit(ADXL345_BW_RATE, 4) > 0;
	}

	public void setLowPower(boolean state) throws IOException {
		this.setRegisterBit(ADXL345_BW_RATE, 4, state ? 1 : 0);
	}

	public double getRate() throws IOException {
		byte[] buff = this.readFrom(ADXL345_BW_RATE, 1);
		buff[0] &= 0xf;
		return Math.pow(2, buff[0] - 6) * 6.25;
	}

	public void setRate(double rate) throws IOException {
		int s;
		int v = (int) (rate / 6.25);
		int r = 0;
		while ((v >>= 1) > 0) {
			r++;
		}
		if (r <= 9) {
			byte[] buff = this.readFrom(ADXL345_BW_RATE, 1);
			s = (r + 6) | (buff[0] & 0xf0);
			this.writeTo(ADXL345_BW_RATE, s);
		}
	}

	public void set_bw(int bw_code) throws IOException {
		if ((bw_code < ADXL345_BW_3) || (bw_code > ADXL345_BW_1600)) {
			throw new IllegalArgumentException("bw_code range overflow");
		} else {
			this.writeTo(ADXL345_BW_RATE, bw_code);
		}
	}

	public int get_bw_code() throws IOException {
		byte[] buff = this.readFrom(ADXL345_BW_RATE, 1);
		return buff[0] & 0xff;
	}

	// Used to check if action was triggered in interrupts
	// Example triggered(interrupts, ADXL345_SINGLE_TAP);
	public boolean triggered(int interrupts, int mask) {
		return ((interrupts >> mask) & 1) > 0;
	}

	/**
	 * Get Source of interrupts 
	 * ADXL345_DATA_READY ADXL345_SINGLE_TAP ADXL345_DOUBLE_TAP ADXL345_ACTIVITY
	 * ADXL345_INACTIVITY ADXL345_FREE_FALL ADXL345_WATERMARK ADXL345_OVERRUNY
	 */
	public int getInterruptSource() throws IOException {
		byte[] buff = this.readFrom(ADXL345_INT_SOURCE, 1);
		return buff[0] & 0xff;
	}

	/**
	 * Source of interrupts 
	 * @param interruptBit
	 * @return
	 * @throws IOException
	 */
	public int getInterruptSource(int interruptBit) throws IOException {
		return this.getRegisterBit(ADXL345_INT_SOURCE, interruptBit);
	}

	/**
	 * Interrupt mapping control 
	 * @param interruptBit
	 * @return
	 * @throws IOException
	 */
	public int getInterruptMapping(int interruptBit) throws IOException {
		return this.getRegisterBit(ADXL345_INT_MAP, interruptBit);
	}

	/**
	 * Set the mapping of an interrupt to pin1 or pin2
	 *  eg: setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,ADXL345_INT2_PIN);
	 * @param interruptBit
	 * @param interruptPin
	 * @throws IOException
	 */
	public void setInterruptMapping(int interruptBit, int interruptPin) throws IOException {
		this.setRegisterBit(ADXL345_INT_MAP, interruptBit, interruptPin);
	}

	public boolean isInterruptEnabled(int interruptBit) throws IOException {
		return this.getRegisterBit(ADXL345_INT_ENABLE, interruptBit) > 0;
	}

	public void setInterrupt(int interruptBit, boolean state) throws IOException {
		this.setRegisterBit(ADXL345_INT_ENABLE, interruptBit, state ? 1 : 0);
	}

	/**
	 * Set bit of the register 
	 * @param regAdress
	 * @param bitPos
	 * @param bit
	 * @throws IOException
	 */
	private void setRegisterBit(int regAdress, int bitPos, int bit) throws IOException {
		byte[] buff = this.readFrom(regAdress, 1);
		if (bit > 0) {
			buff[0] |= (1 << bitPos); // forces nth bit of _b to be 1. all other bits left alone.
		} else {
			buff[0] &= ~(1 << bitPos); // forces nth bit of _b to be 0. all other bits left alone.
		}
		this.writeTo(regAdress, buff[0]);
	}

	/**
	 * Get bit of the register
	 * @param regAdress
	 * @param bitPos
	 * @return
	 * @throws IOException
	 */
	private int getRegisterBit(int regAdress, int bitPos) throws IOException {
		byte[] buff = readFrom(regAdress, 1);
		return ((buff[0] >> bitPos) & 1);
	}
	
	/**
	 *  Writes value to address register on device
	 * @param address
	 * @param val
	 * @throws IOException
	 */
	private void writeTo(int address, int val) throws IOException {
		this._buff[0] = (byte) (val & 0xff);
		this._i2c.write(ADXL345_DEVICE, address, this._buff, 0, 1);
	}

	/**
	 * Reads num bytes starting from address register on device in to _buff array
	 * @param address
	 * @param num
	 * @return
	 * @throws IOException
	 */
	private byte[] readFrom(int address, int num) throws IOException {
		this._i2c.read(ADXL345_DEVICE, address, this._buff, 0, num);
		return this._buff;
	}


}
