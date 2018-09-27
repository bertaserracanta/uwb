/*****************************************************************************/
//	Function:    Header file for class MMC7660 
//  Hardware:    Grove - 3-Axis Digital Accelerometer(±1.5g)
//	Arduino IDE: Arduino-1.0
//	Author:	 FrankieChu		
//	Date: 	 Jan 10,2013
//	Version: v1.0
//	by www.seeedstudio.com
//
//  Modified 2017 for radino32 and radinoL4 compatibility by In-Circuit GmbH
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
/*******************************************************************************/
#ifndef __MMC7660_H__
#define __MMC7660_H__

#define MMA7660_ADDR  0x4C

#define MMA7660_X     0x00          // output value X               (r)
    #define X_ALERT     (0x01)<<6   // if != 0: The register was read at the  
                                    // same time as MMA7660FC was attempting  
                                    // to update the contents. Re-read the 
                                    // register
#define MMA7660_Y     0x01          // output value Y               (r)
    #define Y_ALERT     (0x01)<<6        // if != 0: The register was read at the  
                                    // same time as MMA7660FC was attempting  
                                    // to update the contents. Re-read the 
                                    // register
#define MMA7660_Z     0x02          // output value Z               (r)
    #define Z_ALERT     (0x01)<<6   // if != 0: The register was read at the  
                                    // same time as MMA7660FC was attempting  
                                    // to update the contents. Re-read the 
                                    // register
#define MMA7660_TILT  0x03          // Tilt Status                  (r)
    #define TILT_BAFRO  0x00        // Front/Back -> getBackFrontStatus()
        #define BAFRO_UNKNOWN   0x00    // Unknown front/back-condition
        #define BAFRO_FRONT     0x01    // lying on FRONT
        #define BAFRO_BACK      0x02    // lying on BACK
    
    #define TILT_POLA   0x02        // Left/Right/Down/Up -> getOrientationStatus()
        #define POLA_UNKNOWN    0x00    // Unknown left/right/...-condition
        #define POLA_LEFT       0x01    // in landscape mode to the LEFT
        #define POLA_RIGHT      0x02    // in landscape mode to the RIGHT
        #define POLA_DOWN       0x05    // in vertically inverted orientation
        #define POLA_UP         0x06    // in vertically normal orientation
        
    #define TILT_TAP    0x05        // Tap detection
        #define NO_TAP_DETECTED 0x00    // no tap was detected
        #define TAP_DETECTED    0x01    // a tap was detected
    
    #define TILT_ALERT  (0x01)<<6   // if != 0: The register was read at the  
                                    // same time as MMA7660FC was attempting  
                                    // to update the contents. Re-read the 
                                    // register
    #define TILT_SHAKE   0x07       // Shake detection
        #define NO_SHAKE_DETECTED 0x00    // no shake was detected
        #define SHAKE_DETECTED    0x01    // a shake was detected       

// MMA7660_TILT MASK DEFINITION
// #define TILT_SHAKE_DETECTED    ((SHAKE_DETECTED) << TILT_SHAKE)
// #define TILT_TAP_DETECTED      ((TAP_DETECTED) << TILT_TAP)
// #define TILT_POLA_UNKNOWN      ((POLA_UNKNOWN) << TILT_POLA)
// #define TILT_POLA_LEFT         ((POLA_LEFT) << TILT_POLA)
// #define TILT_POLA_RIGHT        ((POLA_RIGHT) << TILT_POLA)
// #define TILT_POLA_DOWN         ((POLA_DOWN) << TILT_POLA) 
// #define TILT_POLA_UP           ((POLA_UP) << TILT_POLA)
// 
// #define TILT_BAFRO_UNKNOWN     ((BAFRO_UNKNOWN) << TILT_BAFRO)
// #define TILT_BAFRO_FRONT       ((BAFRO_FRONT) << TILT_BAFRO)
// #define TILT_BAFRO_BACK        ((BAFRO_BACK) << TILT_BAFRO)

#define MMA7660_SRST  0x04          // Sample Rate Status Register  (r)
    #define SRST_AMSRS   0x00
    #define SRST_AWSRS   0x01

#define MMA7660_SPCNT 0x05          // Sleep Count Register         (r/w)

#define MMA7660_INTSU 0x06          // Interrupt Setup Register     (r/w)
    #define INT_FBINT   (0x01)<<0        // Front/Back Position change Int.
    #define INT_PLINT   (0x01)<<1        // Up/Down/Right/Left Position Change Int.
    #define INT_PDINT   (0x01)<<2        // Tap-Detection Int.
    #define INT_ASINT   (0x01)<<3        // Exiting Auto-Sleep Int.
    #define INT_GINT    (0x01)<<4        // Automatic Int. on every measurement
    #define INT_SHINTZ  (0x01)<<5        // Shake detected on Z-axis Int.
    #define INT_SHINTY  (0x01)<<6        // Shake detected on Y-axis Int.
    #define INT_SHINTX  (0x01)<<7        // Shake detected on X-axis Int.

// Datasheet Page 17:
// The device must be placed in Standby Mode 
// to change the values of the registers
#define MMA7660_MODE  0x07          // Mode Register                (r/w)
        #define MMA7660_STAND_BY     0x00   // Standby Mode
        #define MMA7660_ACTIVE       0x01   // Active Mode

    #define MODE_TON  0x02          // activate Test Mode - refer to manual
    #define MODE_AWE  0x03          // Auto-Wake Enable
    #define MODE_ASE  0x04          // Auto-Sleep Enable

    #define MODE_SCPS 0x05          // Prescaler divider
        #define PRESC_DIV_1   0x00  // Prescaler divided-by-1
        #define PRESC_DIV_16  0x01  // Prescaler divided-by-16

    #define MODE_IPP  0x06          // Int.-Pin definition:
        #define INT_PIN_OD    0x00  // Open-Drain
        #define INT_PIN_PP    0x01  // Push-Pull

    #define MODE_IAH  0x07          // Int.-Pin behaviour:
        #define INT_PIN_AL    0x00  // Active Low
        #define INT_PIN_AH    0x01  // Active High

#define MMA7660_SR    0x08		    // Auto-Wake and Active Mode Portrait/Land-
                                    // scape Samples per Seconds Register (r/w)
    #define SR_AMSR   0x00
        #define AUTO_SLEEP_120  0x00    // 120 samples per second
        #define AUTO_SLEEP_64   0x01    // 64 samples per second
    	#define AUTO_SLEEP_32   0x02    // 32 samples per second
    	#define AUTO_SLEEP_16	0x03    // 16 samples per second
    	#define AUTO_SLEEP_8	0x04    // 8 samples per second
    	#define AUTO_SLEEP_4	0x05    // 4 samples per second
    	#define AUTO_SLEEP_2	0x06    // 2 samples per second
    	#define AUTO_SLEEP_1	0x07    // 1 samples per second
    
    #define SR_AWSR   0x03
        #define AUTO_WAKE_32    0x00    // 32 samples per second
        #define AUTO_WAKE_16    0x01    // 16 samples per second
        #define AUTO_WAKE_8     0x02    // 8 samples per second
        #define AUTO_WAKE_1     0x03    // 1 samples per second
    
    #define SR_FILT   0x05              // TILT debounce filtering
        #define FILT_OFF        0x00    // no filtering
        #define FILT_2          0x01    // 2 samples
        #define FILT_3          0x02    // 3 samples
        #define FILT_4          0x03    // 4 samples
        #define FILT_5          0x04    // 5 samples
        #define FILT_6          0x05    // 6 samples
        #define FILT_7          0x06    // 7 samples
        #define FILT_8          0x07    // 8 samples
    
#define MMA7660_PDET  0x09          // Tap/Pulse Detection Register (r/w)
    #define TAP_PDTH  0x00          // Tap detection threshold (5 bits)
    #define TAP_XDA   0x05          // Enable x-axis for tap detection
    #define TAP_YDA   0x06          // Enable y-axis for tap detection
    #define TAP_ZDA   0x07          // Enable z-axis for tap detection

#define MMA7660_PD    0x0A          // Tap/Pulse Debounce Count     (r/w)


class MMA7660
{
private:
public:
	void write(uint8_t _register, uint8_t _data);
	uint8_t read(uint8_t _register);

	void init();
	void setMode(uint8_t mode);
    void setPrescalerDivider(uint8_t divider);
        
    void setSampleRateAS(uint8_t rate);        // Auto-Sleep sample rate
    void enableAutoSleep();                    // enable Auto-Sleep
    void disableAutoSleep();                   // disable Auto-Sleep
    void setSleepCounter(uint8_t count);       // Set sleep counter
    uint8_t getSleepCounter();                 // Get sleep counter  
	
    void setSampleRateAW(uint8_t rate);        // Auto-Wake sample rate
    void enableAutoWake();                     // enable Auto-Wake
    void disableAutoWake();                    // disable Auto-Wake
    
    uint8_t getTiltStatus();                           // Get TILT status byte
    uint8_t getFrontBackStatus(uint8_t tiltStatus);    // Get Front/Back status from tiltStatus
    uint8_t getOrientationStatus(uint8_t tiltStatus);  // Get Left/Right/Up/Down status from tiltStatus
    uint8_t getTapDetected(uint8_t tiltStatus);        // Get "Tap Detected" from tiltStatus
    uint8_t getShakeDetected(uint8_t tiltStatus);      // Get "Shake Detected" from tiltStatus
    
    void setTiltFilter(uint8_t filt);          // Set TILT debounce filtering
    void setupTapDetection(uint8_t axis, uint8_t threshold, uint8_t debounceCnt);    // setup tap detection settings  
    
    void enableInterrupt(uint8_t interrupt);   // enable specified interrupt
    void disableInterrupt(uint8_t interrupt);  // disable specified interrupt
    void disableAllInterrupts();               // disable all interrupts
    void clearInterruptStatus();               // clear interrupt status
    
    void setupInterruptPin(uint8_t pinMode, uint8_t pinActive); // define interrupt pin behaviour   
    
    void getXYZ(int8_t *x,int8_t *y,int8_t *z);
	void getAcclemeter(float *ax,float *ay,float *az);
};

#endif
