# FE102M Energy Monitor Calibration Guide

## Introduction

This guide provides detailed instructions for calibrating the SM Energy FE102M Energy Monitor to ensure accurate measurements. The ATM90E26 energy monitoring IC used in the FE102M requires proper calibration to achieve its high accuracy ratings of ±0.1% for active energy and ±0.2% for reactive energy.

## Required Equipment

To properly calibrate the FE102M Energy Monitor, you will need the following equipment:

1. **Reference Voltage Source** - An AC voltage source with known accurate output (preferably traceable to national standards)
2. **Reference Current Source** - A precision AC current source or a known accurate load
3. **Reference Power Meter** - A precision power analyzer with higher accuracy than the FE102M
4. **Digital Multimeter** - For measuring reference values
5. **Variable AC Power Supply** - For testing at different voltage/current levels
6. **Variable Load** - Resistive, inductive, and capacitive loads for testing different power factors
7. **Computer** - With Arduino IDE installed for programming the FE102M
8. **USB Cable** - For connecting to the FE102M

## Calibration Parameters

The ATM90E26 energy monitoring IC has several registers that need to be calibrated:

| Register | Description | Default Value |
|----------|-------------|---------------|
| UgainA   | Voltage RMS Gain | 0xF765 |
| IgainL   | L Line Current RMS Gain | 0x6F4F |
| IgainN   | N Line Current RMS Gain | 0x7A13 |
| Lgain    | L Line Calibration Gain | 0x1D39 |
| Lphi     | L Line Phase Calibration | 0x0000 |
| Ngain    | N Line Calibration Gain | 0x0000 |
| Nphi     | N Line Phase Calibration | 0x0000 |
| Checksum1 | Checksum 1 | 0xAE70 |
| Checksum2 | Checksum 2 | 0xF7BD |

## Calibration Procedure

Before starting the calibration process, ensure the FE102M is properly connected:
- 9-12V DC power supply connected
- Appropriate CT clamp connected
- AC voltage input connected
- USB connected to your computer

Load the calibration sketch (available in the examples folder) to the FE102M using the Arduino IDE. This sketch allows you to read and write calibration registers and view real-time measurements.

### Voltage Calibration

1. **Connect a known reference voltage** to the AC voltage input of the FE102M. This should be within the meter's operating range (< 250V).

2. **Measure the reference voltage** using your precision digital multimeter.

3. **Read the uncalibrated voltage** from the FE102M through the serial monitor.

4. **Calculate the voltage gain calibration factor**:
   ```
   UgainA = (Reference Voltage / Measured Voltage) × Current UgainA Value
   ```

5. **Write the new UgainA value** to the ATM90E26 register using the calibration sketch.

6. **Verify the calibration** by comparing the FE102M reading with your reference meter. The values should now be very close.

7. **Repeat if necessary** to fine-tune the calibration.

### Current Calibration

1. **Connect a known reference current** through the CT clamp. This can be created using a calibrated load.

2. **Measure the reference current** using your precision digital multimeter or power analyzer.

3. **Read the uncalibrated current** from the FE102M through the serial monitor.

4. **Calculate the current gain calibration factor**:
   ```
   IgainL = (Reference Current / Measured Current) × Current IgainL Value
   ```

5. **Write the new IgainL value** to the ATM90E26 register.

6. **Verify the calibration** by comparing the FE102M reading with your reference meter.

7. **Repeat if necessary** to fine-tune the calibration.

8. **Repeat the process for N Line Current** using the IgainN register if using both current channels.

### Power Calibration

1. **Connect both voltage and current** to the FE102M with a known power factor load.

2. **Measure the reference active power** using your power analyzer.

3. **Read the uncalibrated active power** from the FE102M.

4. **Calculate the power gain calibration factor**:
   ```
   Lgain = (Reference Power / Measured Power) × Current Lgain Value
   ```

5. **Write the new Lgain value** to the ATM90E26 register.

6. **Calculate the phase compensation** if there's a significant phase angle difference between voltage and current:
   ```
   Lphi = PhaseCalibrationValue
   ```
   Note: Determining the correct Lphi value may require experimentation based on power factor readings.

7. **Repeat for N Line** if using both current channels, adjusting Ngain and Nphi.

### Checksum Calculation

After modifying any calibration registers, the checksums must be recalculated:

1. **Read the current checksum values** from registers CSOne (0x2C) and CSTwo (0x3B).

2. **The ATM90E26 will automatically calculate the correct checksums** when you read from these registers after changing calibration values.

3. **Write the new checksum values** to the appropriate registers.

## Saving Calibration Values

Once you have determined optimal calibration values, you can save them:

1. **Store values in EEPROM** - The FE102M library includes functions to save values to the onboard 24C256 EEPROM.

2. **Update default values in library** - For permanent calibration, update the default values in the FE102M.cpp file.

Example code for saving to EEPROM:

```cpp
// Save calibration to EEPROM
energyMonitor.saveCalibration();
```

## Verification

After completing calibration, verify accuracy across the full measurement range:

1. **Test at various voltage levels** (e.g., 100V, 200V, 230V)
2. **Test at various current levels** (e.g., 1A, 5A, 10A, max rated current)
3. **Test with different power factors** (e.g., 1.0, 0.8 leading, 0.8 lagging)
4. **Verify energy accumulation** over a period of time

Document your calibration results for future reference.

## Troubleshooting

### Common Calibration Issues

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Unstable readings | Noise or interference | Check wiring, add filtering, ensure proper grounding |
| Non-linear response | Incorrect CT type or configuration | Verify CT specifications, check jumper settings (JP1, JP2) |
| Zero offset issues | Improper offset calibration | Calibrate with no load to set proper zero offsets |
| Gain discrepancy at low vs. high readings | Non-linearity in CT or measurement circuit | Use piece-wise linear compensation in software |
| Temperature drift | Environmental conditions affecting readings | Use the onboard DHT22 for temperature compensation |

### Checking System Status

The ATM90E26 has a System Status register (0x01) that provides diagnostic information:

- Bit 15: Checksum Error
- Bit 14: Oscillator Failure
- Bit 13-10: Reserved
- Bit 9: Voltage Peak Detection
- Bit 8: Current Peak Detection
- Bit 7: Reserved
- Bit 6: Reactive Energy Direction (positive/negative)
- Bit 5: Reactive Power Sign
- Bit 4: Reserved
- Bit 3: Active Energy Direction (positive/negative)
- Bit 2: Active Power Sign
- Bit 1: Reserved
- Bit 0: Low Power Mode

Check this register if you encounter unexpected behavior during calibration.

## Reference Calibration Values

Below are reference calibration values for common CT clamps. These values can serve as starting points for your calibration:

| CT Model | Voltage Range | IgainL Value | UgainA Value | Lgain Value |
|----------|---------------|--------------|--------------|-------------|
| SCT-013-000 (100A) | 220-240V | 0x6F4F | 0xF765 | 0x1D39 |
| SCT-013-030 (30A) | 220-240V | 0x6E49 | 0xF765 | 0x1D39 |
| SCT-013-050 (50A) | 220-240V | 0x7DFB | 0xF765 | 0x1D39 |
| SCT-006 (20A) | 220-240V | 0x706C | 0xF765 | 0x1D39 |
| SCT-010 (80A) | 220-240V | 0x7A34 | 0xF765 | 0x1D39 |
| SCT-016 (120A) | 220-240V | 0x7BAC | 0xF765 | 0x1D39 |
| SCT-024 (200A) | 220-240V | 0x7CC8 | 0xF765 | 0x1D39 |

Note: These values are approximations and will require fine-tuning for your specific installation.

## Calibration Code Example

Below is an example of how to use the calibration functions in the FE102M library:

```cpp
#include <FE102M.h>

// Create FE102M instance
FE102M energyMonitor;

// Calibration values to be set
uint16_t ugainValue = 0xF765;  // Voltage gain
uint16_t igainLValue = 0x6F4F; // L Line current gain
uint16_t igainNValue = 0x7A13; // N Line current gain
uint16_t lgainValue = 0x1D39;  // L Line calibration gain

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("FE102M Energy Monitor - Calibration Example");
  
  // Initialize the energy monitor
  energyMonitor.begin();
  
  // Read current calibration values
  printCurrentCalibration();
  
  // Set new calibration values
  Serial.println("Setting new calibration values...");
  
  // These values would normally be determined through the calibration procedure
  // Write custom calibration values to ATM90E26
  energyMonitor.writeEnergyIC(Ugain, ugainValue);
  energyMonitor.writeEnergyIC(IgainL, igainLValue);
  energyMonitor.writeEnergyIC(IgainN, igainNValue);
  energyMonitor.writeEnergyIC(Lgain, lgainValue);
  
  // Calculate and update checksums
  uint16_t newChecksum1 = energyMonitor.readEnergyIC(CSOne);
  energyMonitor.writeEnergyIC(CSOne, newChecksum1);
  
  uint16_t newChecksum2 = energyMonitor.readEnergyIC(CSTwo);
  energyMonitor.writeEnergyIC(CSTwo, newChecksum2);
  
  // Save calibration to EEPROM for persistence
  energyMonitor.saveCalibration();
  
  // Verify new calibration values
  printCurrentCalibration();
  
  Serial.println("Calibration complete!");
}

void loop() {
  // Read energy data to verify calibration
  energyMonitor.readEnergyData();
  
  // Print current measurements
  Serial.println("------ Current Measurements ------");
  Serial.print("Voltage: ");
  Serial.print(energyMonitor.getLineVoltage(), 2);
  Serial.println(" V");
  
  Serial.print("L Line Current: ");
  Serial.print(energyMonitor.getLineCurrent(), 3);
  Serial.println(" A");
  
  Serial.print("N Line Current: ");
  Serial.print(energyMonitor.getNLineCurrent(), 3);
  Serial.println(" A");
  
  Serial.print("Active Power: ");
  Serial.print(energyMonitor.getActivePower(), 2);
  Serial.println(" W");
  
  Serial.print("Power Factor: ");
  Serial.println(energyMonitor.getPowerFactor(), 3);
  
  Serial.println("--------------------------------");
  
  delay(5000); // Update every 5 seconds
}

// Function to print current calibration values
void printCurrentCalibration() {
  Serial.println("------ Current Calibration Values ------");
  
  Serial.print("Voltage Gain (UgainA): 0x");
  Serial.println(energyMonitor.readEnergyIC(Ugain), HEX);
  
  Serial.print("L Line Current Gain (IgainL): 0x");
  Serial.println(energyMonitor.readEnergyIC(IgainL), HEX);
  
  Serial.print("N Line Current Gain (IgainN): 0x");
  Serial.println(energyMonitor.readEnergyIC(IgainN), HEX);
  
  Serial.print("L Line Calibration Gain (Lgain): 0x");
  Serial.println(energyMonitor.readEnergyIC(Lgain), HEX);
  
  Serial.print("System Status: 0x");
  Serial.println(energyMonitor.getSysStatus(), HEX);
  
  Serial.print("Checksum 1: 0x");
  Serial.println(energyMonitor.readEnergyIC(CSOne), HEX);
  
  Serial.print("Checksum 2: 0x");
  Serial.println(energyMonitor.readEnergyIC(CSTwo), HEX);
  
  Serial.println("--------------------------------------");
}
```

## Advanced Calibration

### Piece-wise Non-linearity Compensation

The ATM90E26 supports piece-wise non-linearity compensation with three current-based segments and two programmable thresholds. This feature allows for more accurate measurements across a wide dynamic range.

To implement piece-wise calibration:

1. Determine the optimal calibration values at different current levels (low, medium, high).
2. Configure the appropriate registers for each segment.
3. Test and validate the calibration across the full current range.

### Temperature Compensation

The FE102M includes a DHT22 temperature sensor that can be used for temperature compensation. The ATM90E26 has a typical temperature coefficient of 6 ppm/°C for its reference voltage.

To implement temperature compensation:

1. Record calibration values at different ambient temperatures.
2. Create a compensation algorithm based on temperature readings.
3. Apply temperature compensation to your measurements in software.

## Conclusion

Proper calibration is essential for accurate energy measurements with the FE102M Energy Monitor. By following this guide and using the provided tools, you can achieve the high precision capabilities of the ATM90E26 energy monitoring IC.

Remember to periodically verify and adjust your calibration as needed, especially if you change CT clamps or modify your installation.

For additional support or questions about calibration, please refer to the ATM90E26 datasheet or contact Microcode Embedded Solutions.
