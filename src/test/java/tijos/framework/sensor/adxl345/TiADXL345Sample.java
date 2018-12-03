package tijos.framework.sensor.adxl345;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.framework.util.Delay;

/**
 * TiADXL345Sample 
 *
 */
public class TiADXL345Sample {

	public static void main(String[] args) {

		System.out.println("TiADXL345Sample.");

		try {
			TiI2CMaster i2c0 = TiI2CMaster.open(0);

			TiADXL345 adxl = new TiADXL345(i2c0);

			adxl.powerOn();

			while (true) {
				
				//read acceleration of gravity 
				double[] xyz_g = adxl.readGxyz();
				double Gx = xyz_g[0];
				double Gy = xyz_g[1];
				double Gz = xyz_g[2];
				
				System.out.println("Gx:" + Gx + " Gy:" + Gy + " Gz:" + Gz);

				Delay.msDelay(1000);
			}

		} catch (IOException ex) {
			ex.printStackTrace();
		}

	}
}
