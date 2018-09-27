/*****************************************************************************/
//	Function:	 Cpp file for class MMC7660 
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
#include <Wire.h>
#include "MMA7660.h"

void MMA7660::write(uint8_t _register, uint8_t _data)
{
	Wire.beginTransmission(MMA7660_ADDR);
	Wire.write(_register);   
	Wire.write(_data);
	Wire.endTransmission();
}
uint8_t MMA7660::read(uint8_t _register)
{
	uint8_t data_read = 0x00;
	Wire.beginTransmission(MMA7660_ADDR);
	Wire.write(_register); 
	// register READ requires repeated START condition
    // so we don't send a STOP condition right now
    Wire.endTransmission(false);
    
	while(Wire.available())
	{
        Wire.read();
    }
    
    Wire.beginTransmission(MMA7660_ADDR);
	Wire.requestFrom(MMA7660_ADDR,1);
	while(Wire.available())
	{
		data_read = Wire.read();
	}
	Wire.endTransmission();
	return data_read;
}

void MMA7660::init()
{
	setMode(MMA7660_STAND_BY);  // standby mode: enable access to registers
    
	setSampleRateAS(AUTO_SLEEP_120);
    setSleepCounter(0x00);
    disableAutoSleep();
    
    setSampleRateAW(AUTO_WAKE_32);
    disableAutoWake();
    
    setupInterruptPin(INT_PIN_OD, INT_PIN_AL);  // open-drain / active-low
    disableAllInterrupts();
    
    setTiltFilter(FILT_OFF);
    setPrescalerDivider(PRESC_DIV_1);
    
    setMode(MMA7660_ACTIVE);    // active mode: measurement activated
}

void MMA7660::setMode(uint8_t mode)
{
    uint8_t reg = read(MMA7660_MODE);
    reg &= ~(0x03<<0); 
    reg |= (mode & 0x03)<<0;
    write(MMA7660_MODE, reg);
}

void MMA7660::setPrescalerDivider(uint8_t divider)
{
    uint8_t reg = read(MMA7660_MODE);
    reg &= ~(0x01<<MODE_SCPS); 
    reg |= (divider & 0x01)<<MODE_SCPS;
    write(MMA7660_MODE, reg);
}

// Auto-Sleep sample rate
void MMA7660::setSampleRateAS(uint8_t rate)
{
	uint8_t reg = read(MMA7660_SR);
    reg &= ~(0x07<<SR_AMSR); 
    reg |= (rate & 0x07)<<SR_AMSR;
    write(MMA7660_SR,reg);
}

// enable Auto-Sleep
void MMA7660::enableAutoSleep()
{
    uint8_t reg = read(MMA7660_MODE); 
    reg |= (0x01)<<MODE_ASE;
    write(MMA7660_MODE, reg);
}

// disable Auto-Sleep
void MMA7660::disableAutoSleep()
{
    uint8_t reg = read(MMA7660_MODE); 
    reg &= ~((0x01)<<MODE_ASE);
    write(MMA7660_MODE, reg);
}

// Set sleep counter
void MMA7660::setSleepCounter(uint8_t count)
{
    write(MMA7660_SPCNT, count);
}

// Get sleep counter
uint8_t MMA7660::getSleepCounter()
{
    return read(MMA7660_SPCNT);
}

// Auto-Wake sample rate
void MMA7660::setSampleRateAW(uint8_t rate)
{
	uint8_t reg = read(MMA7660_SR);
    reg &= ~(0x03<<SR_AWSR); 
    reg |= (rate & 0x03)<<SR_AWSR;
    write(MMA7660_SR,reg);
}

// enable Auto-Wake
void MMA7660::enableAutoWake()
{
    uint8_t reg = read(MMA7660_MODE); 
    reg |= (0x01)<<MODE_AWE;
    write(MMA7660_MODE, reg);
}

// disable Auto-Wake
void MMA7660::disableAutoWake()
{
    uint8_t reg = read(MMA7660_MODE); 
    reg &= ~((0x01)<<MODE_AWE);
    write(MMA7660_MODE, reg);
}

// Get TILT status byte
// Retry if ALERT-bit was set while reading
uint8_t MMA7660::getTiltStatus()
{
    uint8_t tilt = 0;
    
    uint8_t retryCnt = 1;
    do {
        tilt = read(MMA7660_TILT);
    } while ((tilt & TILT_ALERT) && (--retryCnt));
    
    return tilt;
}

// Get Front/Back status from tiltStatus
uint8_t MMA7660::getFrontBackStatus(uint8_t tiltStatus)
{
    return ((tiltStatus>>TILT_BAFRO) & 0x03);
}

// Get Left/Right/Up/Down status from tiltStatus
uint8_t MMA7660::getOrientationStatus(uint8_t tiltStatus)
{
    return ((tiltStatus>>TILT_POLA) & 0x07);
}

// Get "Tap Detected" from tiltStatus
uint8_t MMA7660::getTapDetected(uint8_t tiltStatus)
{
    return ((tiltStatus>>TILT_TAP) & 0x01);
}

// Get "Shake Detected" from tiltStatus
// A shake is detected if an axis' g-load exceeds 1.3g
uint8_t MMA7660::getShakeDetected(uint8_t tiltStatus)
{
    return ((tiltStatus>>TILT_SHAKE) & 0x01);    
}

// setup tap detection settings
// e.g.: setupTapDetection(TAP_XDA | TAP_YDA | TAP_ZDA, 0x0F, 0x0F)
void MMA7660::setupTapDetection(uint8_t axis, uint8_t threshold, uint8_t debounceCnt)
{
    write(MMA7660_PD, debounceCnt);
    write(MMA7660_PDET, (axis & 0xE0) | (threshold & 0x1F));
}

// Set TILT debounce filtering
void MMA7660::setTiltFilter(uint8_t filt)
{
    uint8_t reg = read(MMA7660_SR);
    reg &= ~(0x07<<SR_FILT); 
    reg |= (filt & 0x07)<<SR_FILT;
    write(MMA7660_SR,reg);
}

// enable specified interrupt
void MMA7660::enableInterrupt(uint8_t interrupt)
{
    uint8_t reg = read(MMA7660_INTSU);
    reg |= interrupt;
    write(MMA7660_INTSU,reg);
}

// disable specified interrupt
void MMA7660::disableInterrupt(uint8_t interrupt)
{
    uint8_t reg = read(MMA7660_INTSU);
    reg &= ~(interrupt);
    write(MMA7660_INTSU,reg);
}

// disable all interrupts
void MMA7660::disableAllInterrupts()
{
    write(MMA7660_INTSU, 0x00);
}

// clear interrupt status
// i2c address match clears interrupt status
// so we just send some dummy data via i2c
void MMA7660::clearInterruptStatus()
{
    Wire.beginTransmission(MMA7660_ADDR);
    Wire.write(0x00);
	Wire.endTransmission();    
}

// define interrupt pin behaviour
void MMA7660::setupInterruptPin(uint8_t pinMode, uint8_t pinActive)
{
    uint8_t reg = read(MMA7660_MODE);
    
    reg &= ~(0x01<<MODE_IPP);  
    reg |= (pinMode & 0x01)<<MODE_IPP;
    
    reg &= ~(0x01<<MODE_IAH);  
    reg |= (pinActive & 0x01)<<MODE_IAH;    
    
    write(MMA7660_MODE, reg);
}

// Get int8_t XYZ-values (-31 .. +32)
void MMA7660::getXYZ(int8_t *x,int8_t *y,int8_t *z)
{
	unsigned char val[3];
  	val[0] = val[1] = val[2] = 64;
    
    // datasheet Table 9 / page 14
    // "If the Alert bit is set, the register was read at the same time 
    // as the device was attempting to update the contents. The register 
    // must be read again."
    uint8_t retryCnt = 3;
    do {        
        val[0] = read(MMA7660_X);
        val[1] = read(MMA7660_Y);
        val[2] = read(MMA7660_Z);
    } while (((val[0] & X_ALERT) || (val[1] & Y_ALERT) || (val[2] & Z_ALERT)) && (--retryCnt));
    
    *x = (int8_t)((char)(val[0]<<2))/4;
  	*y = (int8_t)((char)(val[1]<<2))/4;
  	*z = (int8_t)((char)(val[2]<<2))/4;
}
void MMA7660::getAcclemeter(float *ax,float *ay,float *az)
{
	int8_t x,y,z;
	getXYZ(&x,&y,&z);
	*ax = x/21.00;
	*ay = y/21.00;
	*az = z/21.00;
}
