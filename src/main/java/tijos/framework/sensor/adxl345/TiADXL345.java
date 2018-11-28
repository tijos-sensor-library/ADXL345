package tijos.framework.sensor.adxl345;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.framework.util.LittleBitConverter;

class Vector {
	float XAxis;
	float YAxis;
	float ZAxis;
};

class Activites {
	boolean isOverrun;
	boolean isWatermark;
	boolean isFreeFall;
	boolean isInactivity;
	boolean isActivity;
	boolean isActivityOnX;
	boolean isActivityOnY;
	boolean isActivityOnZ;
	boolean isDoubleTap;
	boolean isTap;
	boolean isTapOnX;
	boolean isTapOnY;
	boolean isTapOnZ;
	boolean isDataReady;
};

/**
 * Hello world!
 *
 */
public class TiADXL345 {

	public static final int ADXL345_ADDRESS = 0x53;

	public static final int ADXL345_REG_DEVID = 0x00;
	public static final int ADXL345_REG_THRESH_TAP = 0x1D; // 1
	public static final int ADXL345_REG_OFSX = 0x1E;
	public static final int ADXL345_REG_OFSY = 0x1F;
	public static final int ADXL345_REG_OFSZ = 0x20;
	public static final int ADXL345_REG_DUR = 0x21; // 2
	public static final int ADXL345_REG_LATENT = 0x22; // 3
	public static final int ADXL345_REG_WINDOW = 0x23; // 4
	public static final int ADXL345_REG_THRESH_ACT = 0x24; // 5
	public static final int ADXL345_REG_THRESH_INACT = 0x25; // 6
	public static final int ADXL345_REG_TIME_INACT = 0x26; // 7
	public static final int ADXL345_REG_ACT_INACT_CTL = 0x27;
	public static final int ADXL345_REG_THRESH_FF = 0x28; // 8
	public static final int ADXL345_REG_TIME_FF = 0x29; // 9
	public static final int ADXL345_REG_TAP_AXES = 0x2A;
	public static final int ADXL345_REG_ACT_TAP_STATUS = 0x2B;
	public static final int ADXL345_REG_BW_RATE = 0x2C;
	public static final int ADXL345_REG_POWER_CTL = 0x2D;
	public static final int ADXL345_REG_INT_ENABLE = 0x2E;
	public static final int ADXL345_REG_INT_MAP = 0x2F;
	public static final int ADXL345_REG_INT_SOURCE = 0x30; // A
	public static final int ADXL345_REG_DATA_FORMAT = 0x31;
	public static final int ADXL345_REG_DATAX0 = 0x32;
	public static final int ADXL345_REG_DATAX1 = 0x33;
	public static final int ADXL345_REG_DATAY0 = 0x34;
	public static final int ADXL345_REG_DATAY1 = 0x35;
	public static final int ADXL345_REG_DATAZ0 = 0x36;
	public static final int ADXL345_REG_DATAZ1 = 0x37;
	public static final int ADXL345_REG_FIFO_CTL = 0x38;
	public static final int ADXL345_REG_FIFO_STATUS = 0x39;

	public static final float ADXL345_GRAVITY_SUN = 273.95f;
	public static final float ADXL345_GRAVITY_EARTH = 9.80665f;
	public static final float ADXL345_GRAVITY_MOON = 1.622f;
	public static final float ADXL345_GRAVITY_MARS = 3.69f;
	public static final float ADXL345_GRAVITY_NONE = 1.00f;

	/**
	 * 
	 * adxl345_dataRate_t
	 */

	public static final int ADXL345_DATARATE_3200HZ = 0b1111;
	public static final int ADXL345_DATARATE_1600HZ = 0b1110;
	public static final int ADXL345_DATARATE_800HZ = 0b1101;
	public static final int ADXL345_DATARATE_400HZ = 0b1100;
	public static final int ADXL345_DATARATE_200HZ = 0b1011;
	public static final int ADXL345_DATARATE_100HZ = 0b1010;
	public static final int ADXL345_DATARATE_50HZ = 0b1001;
	public static final int ADXL345_DATARATE_25HZ = 0b1000;
	public static final int ADXL345_DATARATE_12_5HZ = 0b0111;
	public static final int ADXL345_DATARATE_6_25HZ = 0b0110;
	public static final int ADXL345_DATARATE_3_13HZ = 0b0101;
	public static final int ADXL345_DATARATE_1_56HZ = 0b0100;
	public static final int ADXL345_DATARATE_0_78HZ = 0b0011;
	public static final int ADXL345_DATARATE_0_39HZ = 0b0010;
	public static final int ADXL345_DATARATE_0_20HZ = 0b0001;
	public static final int ADXL345_DATARATE_0_10HZ = 0b0000;

	/**
	 * 
	 * adxl345_int_t
	 */
	public static final int ADXL345_INT2 = 0b01;
	public static final int ADXL345_INT1 = 0b00;

	/**
	 * adxl345_activity_t
	 */
	public static final int ADXL345_DATA_READY = 0x07;
	public static final int ADXL345_SINGLE_TAP = 0x06;
	public static final int ADXL345_DOUBLE_TAP = 0x05;
	public static final int ADXL345_ACTIVITY = 0x04;
	public static final int ADXL345_INACTIVITY = 0x03;
	public static final int ADXL345_FREE_FALL = 0x02;
	public static final int ADXL345_WATERMARK = 0x01;
	public static final int ADXL345_OVERRUN = 0x00;

	/**
	 * adxl345_range_t
	 */
	public static final int ADXL345_RANGE_16G = 0b11;
	public static final int ADXL345_RANGE_8G = 0b10;
	public static final int ADXL345_RANGE_4G = 0b01;
	public static final int ADXL345_RANGE_2G = 0b00;

	Vector r;
	Vector n;
	Vector f;
	Activites a;
	int _range;

	/**
	 * TiI2CMaster object
	 */
	private TiI2CMaster i2cMaster;


	public TiADXL345(TiI2CMaster i2c) {
		this.i2cMaster = i2c;
	}
	
	public boolean begin() throws IOException {
		f.XAxis = 0;
		f.YAxis = 0;
		f.ZAxis = 0;

		// Check ADXL345 REG DEVID
		if (fastRegister8(ADXL345_REG_DEVID) != 0xE5) {
			return false;
		}

		// Enable measurement mode (0b00001000)
		writeRegister8(ADXL345_REG_POWER_CTL, (byte) 0x08);

		clearSettings();

		return true;
	}

	// Set Range
	public void setRange(int range) throws IOException {
		// Get actual value register
		byte value = readRegister8(ADXL345_REG_DATA_FORMAT);

		// Update the data rate
		// (&) 0b11110000 (0xF0 - Leave HSB)
		// (|) 0b0000xx?? (range - Set range)
		// (|) 0b00001000 (0x08 - Set Full Res)
		value &= 0xF0;
		value |= range;
		value |= 0x08;

		writeRegister8(ADXL345_REG_DATA_FORMAT, value);
	}

	// Get Range
	public int getRange() throws IOException {
		return readRegister8(ADXL345_REG_DATA_FORMAT) & 0x03;
	}

	// Set Data Rate
	public void setDataRate(int dataRate) throws IOException {
		writeRegister8(ADXL345_REG_BW_RATE, (byte) dataRate);
	}

	// Get Data Rate
	public int getDataRate() throws IOException {
		return readRegister8(ADXL345_REG_BW_RATE) & 0x0F;
	}

	public Vector lowPassFilter(Vector vector) {
		return lowPassFilter(vector, 0.5f);
	}
	// Low Pass Filter
	public Vector lowPassFilter(Vector vector, float alpha) {
		f.XAxis = (vector.XAxis * alpha + (f.XAxis * (1.0f - alpha)));
		f.YAxis = vector.YAxis * alpha + (f.YAxis * (1.0f - alpha));
		f.ZAxis = vector.ZAxis * alpha + (f.ZAxis * (1.0f - alpha));
		return f;
	}

	// Read raw values
	public Vector readRaw() throws IOException {
		r.XAxis = readRegister16(ADXL345_REG_DATAX0);
		r.YAxis = readRegister16(ADXL345_REG_DATAY0);
		r.ZAxis = readRegister16(ADXL345_REG_DATAZ0);
		return r;
	}
	
	public Vector readNormalize() throws IOException {
		return readNormalize(ADXL345_GRAVITY_EARTH);
	}

	// Read normalized values
	public Vector readNormalize(float gravityFactor) throws IOException {
		readRaw();

		// (4 mg/LSB scale factor in Full Res) * gravity factor
		n.XAxis = r.XAxis * 0.004f * gravityFactor;
		n.YAxis = r.YAxis * 0.004f * gravityFactor;
		n.ZAxis = r.ZAxis * 0.004f * gravityFactor;

		return n;
	}

	// Read scaled values
	public Vector readScaled() throws IOException {
		readRaw();

		// (4 mg/LSB scale factor in Full Res)
		n.XAxis = r.XAxis * 0.004f;
		n.YAxis = r.YAxis * 0.004f;
		n.ZAxis = r.ZAxis * 0.004f;

		return n;
	}

	public void clearSettings() throws IOException {
		setRange(ADXL345_RANGE_2G);
		setDataRate(ADXL345_DATARATE_100HZ);

		writeRegister8(ADXL345_REG_THRESH_TAP, (byte) 0x00);
		writeRegister8(ADXL345_REG_DUR, (byte) 0x00);
		writeRegister8(ADXL345_REG_LATENT, (byte) 0x00);
		writeRegister8(ADXL345_REG_WINDOW, (byte) 0x00);
		writeRegister8(ADXL345_REG_THRESH_ACT, (byte) 0x00);
		writeRegister8(ADXL345_REG_THRESH_INACT, (byte) 0x00);
		writeRegister8(ADXL345_REG_TIME_INACT, (byte) 0x00);
		writeRegister8(ADXL345_REG_THRESH_FF, (byte) 0x00);
		writeRegister8(ADXL345_REG_TIME_FF, (byte) 0x00);

		byte value;

		value = readRegister8(ADXL345_REG_ACT_INACT_CTL);
		value &= 0b10001000;
		writeRegister8(ADXL345_REG_ACT_INACT_CTL, value);

		value = readRegister8(ADXL345_REG_TAP_AXES);
		value &= 0b11111000;
		writeRegister8(ADXL345_REG_TAP_AXES, value);
	}

	// Set Tap Threshold (62.5mg / LSB)
	public void setTapThreshold(float threshold) throws IOException {
		byte scaled = (byte) constrain(threshold / 0.0625f, 0, 255);
		writeRegister8(ADXL345_REG_THRESH_TAP, scaled);
	}

	// Get Tap Threshold (62.5mg / LSB)
	public float getTapThreshold() throws IOException {
		return readRegister8(ADXL345_REG_THRESH_TAP) * 0.0625f;
	}

	// Set Tap Duration (625us / LSB)
	public void setTapDuration(float duration) throws IOException {
		byte scaled = (byte) constrain(duration / 0.000625f, 0, 255);
		writeRegister8(ADXL345_REG_DUR, scaled);
	}

	// Get Tap Duration (625us / LSB)
	public float getTapDuration() throws IOException {
		return readRegister8(ADXL345_REG_DUR) * 0.000625f;
	}

	// Set Double Tap Latency (1.25ms / LSB)
	public void setDoubleTapLatency(float latency) throws IOException {
		byte scaled = (byte) constrain(latency / 0.00125f, 0, 255);
		writeRegister8(ADXL345_REG_LATENT, scaled);
	}

	// Get Double Tap Latency (1.25ms / LSB)
	public float getDoubleTapLatency() throws IOException {
		return readRegister8(ADXL345_REG_LATENT) * 0.00125f;
	}

	// Set Double Tap Window (1.25ms / LSB)
	public void setDoubleTapWindow(float window) throws IOException {
		byte scaled = (byte) constrain(window / 0.00125f, 0, 255);
		writeRegister8(ADXL345_REG_WINDOW, scaled);
	}

	// Get Double Tap Window (1.25ms / LSB)
	public float getDoubleTapWindow() throws IOException {
		return readRegister8(ADXL345_REG_WINDOW) * 0.00125f;
	}

	// Set Activity Threshold (62.5mg / LSB)
	public void setActivityThreshold(float threshold) throws IOException {
		byte scaled = (byte) constrain(threshold / 0.0625f, 0, 255);
		writeRegister8(ADXL345_REG_THRESH_ACT, scaled);
	}

	// Get Activity Threshold (65.5mg / LSB)
	public float getActivityThreshold() throws IOException {
		return readRegister8(ADXL345_REG_THRESH_ACT) * 0.0625f;
	}

	// Set Inactivity Threshold (65.5mg / LSB)
	public void setInactivityThreshold(float threshold) throws IOException {
		byte scaled = (byte) constrain(threshold / 0.0625f, 0, 255);
		writeRegister8(ADXL345_REG_THRESH_INACT, scaled);
	}

	// Get Incactivity Threshold (65.5mg / LSB)
	public float getInactivityThreshold() throws IOException {
		return readRegister8(ADXL345_REG_THRESH_INACT) * 0.0625f;
	}

	// Set Inactivity Time (s / LSB)
	public void setTimeInactivity(int time) throws IOException {
		writeRegister8(ADXL345_REG_TIME_INACT, (byte)time);
	}

	// Get Inactivity Time (s / LSB)
	public byte getTimeInactivity() throws IOException {
		return readRegister8(ADXL345_REG_TIME_INACT);
	}

	// Set Free Fall Threshold (65.5mg / LSB)
	public void setFreeFallThreshold(float threshold) throws IOException {
		byte scaled = (byte) constrain(threshold / 0.0625f, 0, 255);
		writeRegister8(ADXL345_REG_THRESH_FF, scaled);
	}

	// Get Free Fall Threshold (65.5mg / LSB)
	public float getFreeFallThreshold() throws IOException {
		return readRegister8(ADXL345_REG_THRESH_FF) * 0.0625f;
	}

	// Set Free Fall Duratiom (5ms / LSB)
	public void setFreeFallDuration(float duration) throws IOException {
		byte scaled = (byte) constrain(duration / 0.005f, 0, 255);
		writeRegister8(ADXL345_REG_TIME_FF, scaled);
	}

	// Get Free Fall Duratiom
	public float getFreeFallDuration() throws IOException {
		return readRegister8(ADXL345_REG_TIME_FF) * 0.005f;
	}

	public void setActivityX(boolean state) throws IOException {
		writeRegisterBit(ADXL345_REG_ACT_INACT_CTL, 6, state);
	}

	public boolean getActivityX() throws IOException {
		return readRegisterBit(ADXL345_REG_ACT_INACT_CTL, 6);
	}

	public void setActivityY(boolean state) throws IOException {
		writeRegisterBit(ADXL345_REG_ACT_INACT_CTL, 5, state);
	}

	public boolean getActivityY() throws IOException {
		return readRegisterBit(ADXL345_REG_ACT_INACT_CTL, 5);
	}

	public void setActivityZ(boolean state) throws IOException {
		writeRegisterBit(ADXL345_REG_ACT_INACT_CTL, 4, state);
	}

	public boolean getActivityZ() throws IOException {
		return readRegisterBit(ADXL345_REG_ACT_INACT_CTL, 4);
	}

	public void setActivityXYZ(int state) throws IOException {
		byte value;

		value = readRegister8(ADXL345_REG_ACT_INACT_CTL);

		if (state > 0) {
			value |= 0b00111000;
		} else {
			value &= 0b11000111;
		}

		writeRegister8(ADXL345_REG_ACT_INACT_CTL, value);
	}

	public void setInactivityX(boolean state) throws IOException {
		writeRegisterBit(ADXL345_REG_ACT_INACT_CTL, 2, state);
	}

	public boolean getInactivityX() throws IOException {
		return readRegisterBit(ADXL345_REG_ACT_INACT_CTL, 2);
	}

	public void setInactivityY(boolean state) throws IOException {
		writeRegisterBit(ADXL345_REG_ACT_INACT_CTL, 1, state);
	}

	public boolean getInactivityY() throws IOException {
		return readRegisterBit(ADXL345_REG_ACT_INACT_CTL, 1);
	}

	public void setInactivityZ(boolean state) throws IOException {
		writeRegisterBit(ADXL345_REG_ACT_INACT_CTL, 0, state);
	}

	public boolean getInactivityZ() throws IOException {
		return readRegisterBit(ADXL345_REG_ACT_INACT_CTL, 0);
	}

	public void setInactivityXYZ(int state) throws IOException {
		byte value;

		value = readRegister8(ADXL345_REG_ACT_INACT_CTL);

		if (state > 0) {
			value |= 0b00000111;
		} else {
			value &= 0b11111000;
		}

		writeRegister8(ADXL345_REG_ACT_INACT_CTL, value);
	}

	public void setTapDetectionX(boolean state) throws IOException {
		writeRegisterBit(ADXL345_REG_TAP_AXES, 2, state);
	}

	public boolean getTapDetectionX() throws IOException {
		return readRegisterBit(ADXL345_REG_TAP_AXES, 2);
	}

	public void setTapDetectionY(boolean state) throws IOException {
		writeRegisterBit(ADXL345_REG_TAP_AXES, 1, state);
	}

	public boolean getTapDetectionY() throws IOException {
		return readRegisterBit(ADXL345_REG_TAP_AXES, 1);
	}

	public void setTapDetectionZ(boolean state) throws IOException {
		writeRegisterBit(ADXL345_REG_TAP_AXES, 0, state);
	}

	public boolean getTapDetectionZ() throws IOException {
		return readRegisterBit(ADXL345_REG_TAP_AXES, 0);
	}

	public void setTapDetectionXYZ(boolean state) throws IOException {
		byte value;

		value = readRegister8(ADXL345_REG_TAP_AXES);

		if (state) {
			value |= 0b00000111;
		} else {
			value &= 0b11111000;
		}

		writeRegister8(ADXL345_REG_TAP_AXES, value);
	}

	public void useInterrupt(int interrupt) throws IOException {
		if (interrupt == 0) {
			writeRegister8(ADXL345_REG_INT_MAP, (byte) 0x00);
		} else {
			writeRegister8(ADXL345_REG_INT_MAP, (byte) 0xFF);
		}

		writeRegister8(ADXL345_REG_INT_ENABLE, (byte) 0xFF);
	}

	public Activites readActivites() throws IOException {
		byte data = readRegister8(ADXL345_REG_INT_SOURCE);

		a.isOverrun = ((data >> ADXL345_OVERRUN) & 1) > 0 ? true : false;
		a.isWatermark = ((data >> ADXL345_WATERMARK) & 1) > 0 ? true : false;
		a.isFreeFall = ((data >> ADXL345_FREE_FALL) & 1) > 0 ? true : false;
		a.isInactivity = ((data >> ADXL345_INACTIVITY) & 1) > 0 ? true : false;
		a.isActivity = ((data >> ADXL345_ACTIVITY) & 1) > 0 ? true : false;
		a.isDoubleTap = ((data >> ADXL345_DOUBLE_TAP) & 1) > 0 ? true : false;
		a.isTap = ((data >> ADXL345_SINGLE_TAP) & 1) > 0 ? true : false;
		a.isDataReady = ((data >> ADXL345_DATA_READY) & 1) > 0 ? true : false;

		data = readRegister8(ADXL345_REG_ACT_TAP_STATUS);

		a.isActivityOnX = ((data >> 6) & 1) > 0 ? true : false;
		a.isActivityOnY = ((data >> 5) & 1) > 0 ? true : false;
		a.isActivityOnZ = ((data >> 4) & 1) > 0 ? true : false;
		a.isTapOnX = ((data >> 2) & 1) > 0 ? true : false;
		a.isTapOnY = ((data >> 1) & 1) > 0 ? true : false;
		a.isTapOnZ = ((data >> 0) & 1) > 0 ? true : false;

		return a;
	}

	private int constrain(float x, int i, int j) {
		if (x > j)
			return j;

		if (x < i)
			return i;

		return (int) x;
	}

	// Write byte to register
	private void writeRegister8(int reg, byte value) throws IOException {
		byte[] data = new byte[2];
		data[0] = (byte) reg;
		data[1] = value;

		i2cMaster.write(ADXL345_ADDRESS, data, 0, 2);
	}

	// Read byte to register
	private byte fastRegister8(int reg) throws IOException {
		byte[] data = new byte[1];
		i2cMaster.read(ADXL345_ADDRESS, reg, data, 0, 1);

		return data[0];
	}

	// Read byte from register
	private byte readRegister8(int reg) throws IOException {
		byte[] data = new byte[1];
		data[0] = (byte) reg;

		i2cMaster.write(ADXL345_ADDRESS, data, 0, 1);
		i2cMaster.read(ADXL345_ADDRESS, data, 0, 1);

		return data[0];
	}

	// Read word from register
	private int readRegister16(int reg) throws IOException {
		byte[] data = new byte[2];
		data[0] = (byte) reg;

		i2cMaster.write(ADXL345_ADDRESS, data, 0, 1);
		i2cMaster.read(ADXL345_ADDRESS, data, 0, 2);

		return LittleBitConverter.ToUInt16(data, 0);
	}

	private void writeRegisterBit(int reg, int pos, boolean state) throws IOException {
		byte value;
		value = readRegister8(reg);

		if (state) {
			value |= (1 << pos);
		} else {
			value &= ~(1 << pos);
		}

		writeRegister8(reg, value);
	}

	private boolean readRegisterBit(int reg, int pos) throws IOException {
		byte value = readRegister8(reg);
		int val = ((value >> pos) & 1);

		if (val > 0)
			return true;

		return false;
	}

	public static void main(String[] args) {
		System.out.println("Hello World!");
	}
}
