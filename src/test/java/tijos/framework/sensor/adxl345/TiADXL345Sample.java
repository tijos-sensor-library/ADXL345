package tijos.framework.sensor.adxl345;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.framework.util.Delay;

/**
 * Hello world!
 *
 */
public class TiADXL345Sample
{
	TiADXL345 accelerometer;
	
	public TiADXL345Sample(TiI2CMaster i2c)
	{
		accelerometer = new TiADXL345(i2c);
	}

	public void setup()  throws IOException
	{
	  // Initialize ADXL345
	  System.out.println("Initialize ADXL345");

	  
	  
	  if (!accelerometer.begin())
	  {
	    System.out.println("Could not find a valid ADXL345 sensor, check wiring!");
	    Delay.msDelay(500);
	  }

	  // Values for Activity and Inactivity detection
	  accelerometer.setActivityThreshold(2.0f);    // Recommended 2 g
	  accelerometer.setInactivityThreshold(2.0f);  // Recommended 2 g
	  accelerometer.setTimeInactivity(5);         // Recommended 5 s

	  // Set activity detection only on X,Y,Z-Axis
	  accelerometer.setActivityXYZ(1);         // Check activity on X,Y,Z-Axis
	  // or
	  // accelerometer.setActivityX(1);        // Check activity on X_Axis
	  // accelerometer.setActivityY(1);        // Check activity on Y-Axis
	  // accelerometer.setActivityZ(1);        // Check activity on Z-Axis

	  // Set inactivity detection only on X,Y,Z-Axis
	  accelerometer.setInactivityXYZ(1);       // Check inactivity on X,Y,Z-Axis
	  // or
	  // accelerometer.setInactivityX(1);      // Check inactivity on X_Axis
	  // accelerometer.setInactivityY(1);      // Check inactivity on Y-Axis
	  // accelerometer.setInactivityZ(1);      // Check inactivity on Z-Axis

	  // Select INT 1 for get activities
	  accelerometer.useInterrupt(TiADXL345.ADXL345_INT1);

	  // Check settings
	  checkSetup();
	}

	private void checkSetup() throws IOException
	{
	  System.out.print("Activity Threshold = "); System.out.println(accelerometer.getActivityThreshold());
	  System.out.print("Inactivity Threshold = "); System.out.println(accelerometer.getInactivityThreshold());
	  System.out.print("Time Inactivity = "); System.out.println(accelerometer.getTimeInactivity());

	  System.out.print("Look activity on axis = "); 
	  if (accelerometer.getActivityX()) { System.out.print(" X "); }
	  if (accelerometer.getActivityY()) { System.out.print(" Y "); }
	  if (accelerometer.getActivityZ()) { System.out.print(" Z "); }
	  System.out.println();

	  System.out.print("Look inactivity on axis = "); 
	  if (accelerometer.getInactivityX()) { System.out.print(" X "); }
	  if (accelerometer.getInactivityY()) { System.out.print(" Y "); }
	  if (accelerometer.getInactivityZ()) { System.out.print(" Z "); }
	  System.out.println();  
	}

	public void loop() throws IOException
	{
	  // Read values for activities
	  Delay.msDelay(50);
	  Vector norm = accelerometer.readNormalize();

	  // Read activities
	  Activites activ = accelerometer.readActivites();

	  if (activ.isActivity)
	  {
	    System.out.println("Activity Detected");
	  }

	  if (activ.isInactivity)
	  {
	    System.out.println("Inactivity Detected");
	  }
	}

    public static void main( String[] args )
    {
        System.out.println( "Hello World!" );
        
        try {
            int i2cPort0 = 0;
        	TiI2CMaster i2c0 = TiI2CMaster.open(i2cPort0);

        	TiADXL345Sample sample = new TiADXL345Sample(i2c0);
        	
        	sample.setup();
        	
        	while(true)
        	{
        		sample.loop();
        		
        		Delay.msDelay(1000);
        	}
        	

        }
        catch(Exception ex) {
        	
        }
        
    }
}
