/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The ICM40627 is a sensor hub with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include "ICM40627.h"
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define INV_ABS(x) (((x) < 0) ? -(x) : (x))

ICM40627::ICM40627()
{
  
}


uint8_t ICM40627::getChipID()
{
  uint8_t c = this->readByte(ICM40627_ADDRESS, ICM40627_WHO_AM_I);
  return c;
}

///@return 0- success, 1 - error 
bool ICM40627::whoamiCheck()
{
  uint8_t id = getChipID();
  uint8_t defautValue = 0x4E;
  if (id != defautValue) {log_e("Wrong chip ID"); return true;}

  return false;
}

uint8_t ICM40627::readByte(uint8_t address, uint8_t subAddress){
    uint8_t data = 0;
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission();
    Wire.requestFrom(address, 1);
    if (Wire.available())
    {
        data = (uint8_t)Wire.read();
    }
    return data;
}


void ICM40627::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);   
  Wire.write(subAddress);            
  Wire.endTransmission(false);       
  uint8_t i = 0;
  Wire.requestFrom(address, count);  
  while (Wire.available()) 
  {
        dest[i++] = Wire.read();
  }  
}


void ICM40627::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data){
    Wire.beginTransmission(devAddr);
    Wire.write(regAddr);
    Wire.write(data);
    Wire.endTransmission();
}



float ICM40627::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      _aRes = 2.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_4G:
      _aRes = 4.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_8G:
      _aRes = 8.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_16G:
      _aRes = 16.0f / 32768.0f;
      return _aRes;
      break;
  }
}

float ICM40627::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
    case GFS_15_125DPS:
      _gRes = 15.125f / 32768.0f;
      return _gRes;
      break;
    case GFS_31_25DPS:
      _gRes = 31.25f / 32768.0f;
      return _gRes;
      break;
    case GFS_62_5DPS:
      _gRes = 62.5f / 32768.0f;
      return _gRes;
      break;
    case GFS_125DPS:
      _gRes = 125.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_250DPS:
      _gRes = 250.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_500DPS:
      _gRes = 500.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_1000DPS:
      _gRes = 1000.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_2000DPS:
      _gRes = 2000.0f / 32768.0f;
      return _gRes;
      break;
  }
}

void ICM40627::reset()
{
  // reset device
  uint8_t temp = this->readByte(ICM40627_ADDRESS, ICM40627_DEVICE_CONFIG);
  this->writeByte(ICM40627_ADDRESS, ICM40627_DEVICE_CONFIG, temp | 0x01); // Set bit 0 to 1 to reset ICM40627
  do {
    log_i("Waiting for reset to ready...");
    delay(10); // Wait for all registers to reset
  }
  while (CHECK_BIT(this->readByte(ICM40627_ADDRESS, ICM40627_INT_STATUS), 4));
  log_i("Reset performed correctly");
}

uint8_t ICM40627::status()
{
  // reset device
  uint8_t temp = this->readByte(ICM40627_ADDRESS, ICM40627_INT_STATUS);
  return temp;
}

void ICM40627::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR)
{
    uint8_t temp = this->readByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0);  
    uint8_t mask = 0x0F;
    this->writeByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0, temp & ~(mask)); // turn off accelerometer and gyroscope 
    delay(10);
    
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_GYRO_CONFIG0);
    this->writeByte(ICM40627_ADDRESS, ICM40627_GYRO_CONFIG0, temp | GODR | Gscale << 5); // gyro full scale and data rate
    delay(10);

    temp = this->readByte(ICM40627_ADDRESS, ICM40627_ACCEL_CONFIG0);
    this->writeByte(ICM40627_ADDRESS, ICM40627_ACCEL_CONFIG0, temp | AODR | Ascale << 5); // set accel full scale and data rate
    delay(10);

    temp = this->readByte(ICM40627_ADDRESS, ICM40627_INT_CONFIG);
    this->writeByte(ICM40627_ADDRESS, ICM40627_INT_CONFIG, temp | 0x18 | 0x03 ); // set both interrupts active high, push-pull, pulsed
    delay(10);

    temp = this->readByte(ICM40627_ADDRESS, ICM40627_INT_CONFIG1);
    this->writeByte(ICM40627_ADDRESS, ICM40627_INT_CONFIG1, temp & ~(0x10) ); // set bit 4 to zero for proper function of INT1 and INT2
    delay(10);

   
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp | 0x04 ); // select Bank 4
    delay(10);

    temp = this->readByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG5);
    mask = 0b00000111;
    temp &= ~mask;
    // this->writeByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG5, temp | 0x00);// select unitary mounting matrix
    this->writeByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG5, temp | 0x00);// select unitary mounting matrix

    delay(10); 

    temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp & ~(0x07) ); // select Bank 0
    delay(10);

    temp = this->readByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG0); 
    this->writeByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG0, (temp & ~(0x80)) | 0x02); // disable DMP power save mode, DMP 50Hz
    delay(10);


     // //* Initialize Sensor
    // Set accelerometer ODR (Register 0x50h in Bank 0)
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_ACCEL_CONFIG0); 
    this->writeByte(ICM40627_ADDRESS, ICM40627_ACCEL_CONFIG0, temp | (0x07));
    delay(10);
    // Set Accel to Low Power mode (Register 0x4Eh in Bank 0) ACCEL_MODE = 2 
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0); 
    this->writeByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0, temp | (0x02));
    delay(10);
    // Set (Register 0x4Dh in Bank 0), ACCEL_LP_CLK_SEL = 0, for low power mode
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_INTF_CONFIG1); 
    this->writeByte(ICM40627_ADDRESS, ICM40627_INTF_CONFIG1, temp & ~(0x08));
    delay(10);
    
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0);
    mask = 0x0F;
    this->writeByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0, temp & ~(mask) | 0x0E); // enable gyro in low noise mode and accelerometer in low power mode
    delay(50); 

    log_i("ICM40627 init performed correctly");
}

void ICM40627::readData(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  this->readBytes(ICM40627_ADDRESS, ICM40627_TEMP_DATA1, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;

}




void ICM40627::initR2SW(void){
    //* Hardware initialization

    // turn off accelerometer and gyroscope 
    uint8_t temp = this->readByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0);
    uint8_t mask = 0x0F;
    this->writeByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0, temp & ~(mask)); 
    delay(10);

    // delay(10);
    //* Initialize Sensor in a typical configuration
    // Set accelerometer ODR (Register 0x50h in Bank 0)
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_ACCEL_CONFIG0); 
    this->writeByte(ICM40627_ADDRESS, ICM40627_ACCEL_CONFIG0, temp | (0x07));
    delay(10);
    // Set Accel to Low Power mode (Register 0x4Eh in Bank 0) ACCEL_MODE = 2 
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0); 
    this->writeByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0, temp | (0x02));
    delay(10);
    // Set (Register 0x4Dh in Bank 0), ACCEL_LP_CLK_SEL = 0, for low power mode
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_INTF_CONFIG1); 
    this->writeByte(ICM40627_ADDRESS, ICM40627_INTF_CONFIG1, temp & ~(0x08));
    delay(10);
    // Set DMP ODR (Register 0x56h in Bank 0)
    // 50hz DMP ODR in default 
    //* Initialize APEX hardware

    temp = this->readByte(ICM40627_ADDRESS, ICM40627_SIGNAL_PATH_RESET);
    this->writeByte(ICM40627_ADDRESS, ICM40627_SIGNAL_PATH_RESET, temp | (0x20));
    delay(10);

    //* Select bank 4 
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp | 0x04 );
    delay(10);

    // configure SLEEP_TIME_OUT
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG4);
    mask = 0b00111000;
    temp &= ~mask;
    this->writeByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG4, temp | 0x38);
    delay(10);
    // configure MOUNTING_MATRIX
    // use setted in init method
    // configure SLEEP_GESTURE_DELAY
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG6);
    mask = 0b00000111;
    temp &= ~mask;
    this->writeByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG6, temp | 0x02);
    delay(10);
    //* Select bank 0
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp & ~(0x07) );
    delay(10); 
    // Set DMP_INIT_EN to 1 (Register 0x4Bh in Bank 0)
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_SIGNAL_PATH_RESET);
    this->writeByte(ICM40627_ADDRESS, ICM40627_SIGNAL_PATH_RESET, temp | (0x40));
    delay(10);
    // Enable Raise to Wake/Sleep, source for INT1 by setting bit 2,1 in register INT_SOURCE6 (Register
    // 0x4Dh in Bank 4) to 1.
    //* Select bank 4 
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp | 0x04 );
    delay(10);
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_INT_SOURCE6);
    this->writeByte(ICM40627_ADDRESS, ICM40627_INT_SOURCE6, temp | (0x6));
    delay(50);
    //* Select bank 0
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp & ~(0x07) );
    delay(10); 
    // Turn on Raise to Wake/Sleep feature by setting R2W_EN to 1 (Register 0x56h in Bank 0)
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG0);
    this->writeByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG0, temp | (0x8));
    delay(50);

    // turn on gyroscope and accelerometer  
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0);
    mask = 0x0F;
    this->writeByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0, temp & ~(mask) | 0x0E); 
    delay(50); 
}

void ICM40627::setWOM(){

    // turn off accelerometer and gyroscope 
    uint8_t temp = this->readByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0);
    uint8_t mask = 0x0F;
    this->writeByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0, temp & ~(mask)); 
    delay(10); 

    // Set accelerometer ODR (Register 0x50h in Bank 0)
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_ACCEL_CONFIG0); 
    this->writeByte(ICM40627_ADDRESS, ICM40627_ACCEL_CONFIG0, temp | (0x09));
    delay(10);
    // Set Accel to Low Power mode (Register 0x4Eh in Bank 0) ACCEL_MODE = 2 
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0); 
    this->writeByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0, temp | (0x02));
    delay(10);
    // Set (Register 0x4Dh in Bank 0), ACCEL_LP_CLK_SEL = 0, for low power mode
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_INTF_CONFIG1); 
    this->writeByte(ICM40627_ADDRESS, ICM40627_INTF_CONFIG1, temp & ~(0x08));
    delay(10);

    //* Select bank 4 
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp | 0x04 );
    delay(10);

    // Set WOM_X_TH to 98 (Register 0x4Ah in Bank 4)
    this->writeByte(ICM40627_ADDRESS, ICM40627_ACCEL_WOM_X_THR,0x10);
    delay(10);
    // Set WOM_Y_TH to 98 (Register 0x4Bh in Bank 4)
    this->writeByte(ICM40627_ADDRESS, ICM40627_ACCEL_WOM_Y_THR, 0x10);
    delay(10);
    // Set WOM_Z_TH to 98 (Register 0x4Ch in Bank 4)
    this->writeByte(ICM40627_ADDRESS, ICM40627_ACCEL_WOM_Z_THR, 0x10);
    delay(10);

    //* Select bank 0
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp & ~(0x07) );
    delay(10); 
    // Enable all 3 axes as WOM sources for INT1 by setting bits 2:0 in register INT_SOURCE1 (Register
    // 0x66h in Bank 0) to 1. Or if INT2 is selected for WOM, enable all 3 axes as WOM sources by
    // setting bits 2:0 in register INT_SOURCE4 (Register 0x69h in Bank 0) to 1
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_INT_SOURCE1); 
    mask = 0x07;
    uint8_t clearedReg = temp & ~mask;
    this->writeByte(ICM40627_ADDRESS, ICM40627_INT_SOURCE1, temp | 0x07);
    delay(50);

    // Turn on WOM feature by setting WOM_INT_MODE to 0, WOM_MODE to 1, SMD_MODE to 1 (Register 0x56h in Bank 0)
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_SMD_CONFIG); 
    mask = 0x0F;
    clearedReg = temp & ~mask;
    this->writeByte(ICM40627_ADDRESS, ICM40627_SMD_CONFIG, clearedReg | 0x05);
    delay(10);

    // turn on gyroscope and accelerometer  
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0);
    mask = 0x0F;
    this->writeByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0, temp & ~(mask) | 0x0E); 
    delay(50); 
}

/// @brief This is specific function which allows to divide 3 signed flaot values and store them into 3 registers
///         with resolution of 1./32. It gives as a max stored value +/- 64
/// @param reg1 register that contains 8 lower bits of val1
/// @param reg2 register which half of it contains some data from val1 and some data from val2
/// @param reg3 register that contains 8 lower bitst of val2
/// @param val1 value to be stored in 1.5 of 8 bit register
/// @param val2 value to be stored in 1.5 of 8 bit register
void ICM40627::getRegistersValuesForOffsets(unsigned char *reg1, unsigned char *reg2, unsigned char *reg3,
                        float val1, float val2, float resolution1,float resolution2){
    
    int fConvertedValue = fabs(val1/resolution1);
    int sConvertedValue = fabs(val2/resolution2);

    *reg1 = fConvertedValue;
    *reg2 = (fConvertedValue>>8)&0x07;
    if(val1<0.0){*reg2|=0x08;}

    *reg2 = (sConvertedValue>>4)&0x70;
    if(val2<0.0){*reg2 |= 0x80;}
    *reg3 = sConvertedValue;
}

void ICM40627::setOffsets(float ax, float ay, float az, float gx, float gy, float gz){

    uint8_t offsetUser0 = 0x00;
    uint8_t offsetUser1 = 0x00;
    uint8_t offsetUser2 = 0x00;
    uint8_t offsetUser3 = 0x00;
    uint8_t offsetUser4 = 0x00;
    uint8_t offsetUser5 = 0x00;
    uint8_t offsetUser6 = 0x00;
    uint8_t offsetUser7 = 0x00;
    uint8_t offsetUser8 = 0x00;
    // check offsets value if their are in given range 
    // +/- 64 dps
    // +/- 1 g 

    // getRegistersValuesForOffsets(&offsetUser0, &offsetUser1, &offsetUser2, gx, gy, resolutionG, resolutionG);
    // getRegistersValuesForOffsets(&offsetUser3, &offsetUser4, &offsetUser5, gz, ax, resolutionG, resolutionA);
    // getRegistersValuesForOffsets(&offsetUser6, &offsetUser7, &offsetUser8, ay, az, resolutionA, resolutionA);
    
    //* Select bank 4 
    uint8_t temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp | 0x04 );
    delay(10);


    this->writeByte(ICM40627_ADDRESS, ICM40627_OFFSET_USER0, offsetUser0);delay(1);
    this->writeByte(ICM40627_ADDRESS, ICM40627_OFFSET_USER1, offsetUser1);delay(1);
    this->writeByte(ICM40627_ADDRESS, ICM40627_OFFSET_USER2, offsetUser2);delay(1);
    this->writeByte(ICM40627_ADDRESS, ICM40627_OFFSET_USER3, offsetUser3);delay(1);
    this->writeByte(ICM40627_ADDRESS, ICM40627_OFFSET_USER4, offsetUser4);delay(1);
    this->writeByte(ICM40627_ADDRESS, ICM40627_OFFSET_USER5, offsetUser5);delay(1);
    this->writeByte(ICM40627_ADDRESS, ICM40627_OFFSET_USER6, offsetUser6);delay(1);
    this->writeByte(ICM40627_ADDRESS, ICM40627_OFFSET_USER7, offsetUser7);delay(1);
    this->writeByte(ICM40627_ADDRESS, ICM40627_OFFSET_USER8, offsetUser8);delay(1);


    //* Select bank 0
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp & ~(0x07) );

}


void ICM40627::initPedometer(){
    

    //*Initialize APEX hardware
    // Set DMP_MEM_RESET_EN to 1 (Register 0x4Bh in Bank 0)
    uint8_t temp = this->readByte(ICM40627_ADDRESS, ICM40627_SIGNAL_PATH_RESET);
    this->writeByte(ICM40627_ADDRESS, ICM40627_SIGNAL_PATH_RESET, temp | 0x20);
    delay(10);

    //* Select bank 4 
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp | 0x04 );
    delay(10);

    // Set LOW_ENERGY_AMP_TH_SEL to 10 (Register 0x40h in Bank 4)
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG1);
    this->writeByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG1, temp | 0xA0);
    delay(10);
    // Set PED_AMP_TH_SEL to 8 (Register 0x41h in Bank 4) 
    // Set PED_STEP_CNT_TH_SEL to 5 (Register 0x41h in Bank 4)
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG2);
    this->writeByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG2, temp | 0x80 | 0x05);
    delay(10);

    // Set PED_HI_EN_TH_SEL to 1 (Register 0x42h in Bank 4)
    // Set PED_SB_TIMER_TH_SEL to 4 (Register 0x42h in Bank 4)
    // Set PED_STEP_DET_TH_SEL to 2 (Register 0x42h in Bank 4)
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG3);
    this->writeByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG3, temp | 0x01 | 0x10 | 0x40);
    delay(10);
    
    // Set SENSITIVITY_MODE to 0 (Register 0x48h in Bank 4)
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG9);
    this->writeByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG9, temp & ~(0x01));
    delay(10);
    //* Select bank 0
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp & ~(0x07) );
    delay(10); 

    // Set DMP_INIT_EN to 1 (Register 0x4Bh in Bank 0)
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_SIGNAL_PATH_RESET);
    this->writeByte(ICM40627_ADDRESS, ICM40627_SIGNAL_PATH_RESET, temp | (0x40));
    delay(50);

    // Enable STEP detection, source for INT1 by setting bit 5 in register INT_SOURCE6 (Register 0x4Dh
    // in Bank 4) to 1. Or if INT2 is selected for STEP detection, enable STEP detection source by setting
    // bit 5 in register INT_SOURCE7 (Register 0x4Eh in Bank 4) to 1.
    //* Select bank 4 
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp | 0x04 );
    delay(10);

    temp = this->readByte(ICM40627_ADDRESS, ICM40627_INT_SOURCE7);
    this->writeByte(ICM40627_ADDRESS, ICM40627_INT_SOURCE7, temp | (0x20));
    delay(50);
    

    //* Select bank 0
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp & ~(0x07) );
    delay(10); 
    // Turn on Pedometer feature by setting PED_ENABLE to 1 (Register 0x56h in Bank 0)
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG0);
    this->writeByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG0, temp | (0x20));
    delay(50);
}


void ICM40627::initTapDetection(){
    // Set filter settings as follows: ACCEL_DEC2_M2_ORD = 2 (Register 0x53h in Bank 0);
    uint8_t temp = this->readByte(ICM40627_ADDRESS, ICM40627_ACCEL_CONFIG1);
    this->writeByte(ICM40627_ADDRESS, ICM40627_ACCEL_CONFIG1, temp | (0x02));
    delay(10);
    // ACCEL_UI_FILT_BW = 4 (Register 0x52h in Bank 0)
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_GYRO_ACCEL_CONFIG0);
    this->writeByte(ICM40627_ADDRESS, ICM40627_GYRO_ACCEL_CONFIG0, temp | (0x04));
    // delay(10);
    //* Initialize APEX hardware
    //* Select bank 4 
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp | 0x04 );
    delay(10);
    // Set TAP_TMAX to 2 (Register 0x47h in Bank 4)     -> 01
    // Set TAP_TMIN to 3 (Register 0x47h in Bank 4)     -> 011 
    // Set TAP_TAVG to 3 (Register 0x47h in Bank 4)     -> 11
    // 0b00111011 -> 0x3B
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG8);
    this->writeByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG8, temp | 0x3B);
    delay(10);
    // Set TAP_MIN_JERK_THR to 17 (Register 0x46h in Bank 4)
    // Set TAP_MAX_PEAK_TOL to 2 (Register 0x46h in Bank 4)
    // 0b01000110 -> 0x46
    // 0b01100110 -> 0x66
    // 0b01110110 -> 0x76
    // 0b01110111 -> 0x77
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG7);
    this->writeByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG7, temp | 0x46);
    delay(10);
    // Enable TAP source for INT1 by setting bit 0 in register INT_SOURCE6 (Register 0x4Dh in Bank 4) to 1.
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_INT_SOURCE6);
    this->writeByte(ICM40627_ADDRESS, ICM40627_INT_SOURCE6, temp | 0x01);
    delay(50);
    //* Select bank 0
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, temp & ~(0x07) );
    delay(10); 
    // Turn on TAP feature by setting TAP_ENABLE to 1 (Register 0x56h in Bank 0).
    temp = this->readByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG0);
    this->writeByte(ICM40627_ADDRESS, ICM40627_APEX_CONFIG0, temp | 0x40);
    delay(10); 
}


void ICM40627::sleepSensor(){
    // turn off accelerometer and gyroscope 
    uint8_t temp = this->readByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0);
    uint8_t mask = 0x0F;
    this->writeByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0, temp & ~(mask)); 
    delay(10); 
}

void ICM40627::wakeUpSensor(){
    // turn on gyroscope and accelerometer  
    uint8_t temp = this->readByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0);
    uint8_t mask = 0x0F;
    this->writeByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0, temp & ~(mask) | 0x0E); 
    delay(50); 
}

int ICM40627::readWakeSleepStatus(){

    uint8_t int_status3 = this->readByte(ICM40627_ADDRESS, ICM40627_INT_STATUS3);
    uint8_t apex_data4 = this->readByte(ICM40627_ADDRESS,ICM40627_APEX_DATA4);

    int posTap = 0;
    int posTapNum0 = 3;
    int posTapNum1 = 4;
    int posTapAxis0 = 1;
    int posTapAxis1 = 2;
    int posTapDir = 0;

    if(CHECK_BIT(int_status3, posTap) // tap interrupt
        and (CHECK_BIT(apex_data4, posTapNum1) and !CHECK_BIT(apex_data4, posTapNum0)) // duble tap
        and (!CHECK_BIT(apex_data4, posTapAxis1) and CHECK_BIT(apex_data4, posTapAxis0))) // Y-axis
        {return 1;}
    else return 0;

}

void ICM40627::readPedometerStatus(uint16_t *result){
    uint8_t rawData[2];
    this->readBytes(ICM40627_ADDRESS, ICM40627_APEX_DATA0, 2, &rawData[0]);  
    *result = ((int16_t)rawData[1] << 8) | rawData[0]; 
    log_v("Steps: %d\n",*result);
}

/**
 * @brief IMU self test method that is available from onboard software. 
 * 
 * @return 0 if successful self test 
 * @return 1 if failed self test
 */
bool ICM40627::selfTest(){
    bool st_gyro = runGyroSelfTest();
    if(st_gyro) {
        log_e("gyro failed self test");
        return 1;
    }
    bool st_accel = runAccelSelfTest();
    if(st_accel){
        log_e("accel failed self test");
        return 1;
    }
    if (!st_gyro and !st_accel){
        log_i("imu passed self test");
        return 0;
    }
}

/**
 * @brief Method to self test gyroscope
 * 
 * @return 0 if self test passed 
 * @return 1 if self test failed 
 */
bool ICM40627::runGyroSelfTest(){
    int status = 0;
	uint8_t data;
	uint8_t bank;
    
	int32_t STG_OFF[3], STG_ON[3];
	uint32_t STG_response[3];

	uint8_t ST_code_regs[3];
	uint32_t STG_OTP[3];

	int i = 0;
	
	uint32_t gyro_sensitivity_1dps;

	/* Set gyro configuration */
	// status |= inv_icm406xx_read_reg(ICM40627_ADDRESS, ICM40627_GYRO_CONFIG0, 1, &data);
    data = this->readByte(ICM40627_ADDRESS, ICM40627_GYRO_CONFIG0);
	data &= ~BIT_GYRO_CONFIG0_FS_SEL_MASK;
	data &= ~BIT_GYRO_CONFIG0_ODR_MASK;
	data |= ST_GYRO_FSR;
	data |= ST_GYRO_ODR;
	// status |= inv_icm406xx_write_reg(ICM40627_ADDRESS, ICM40627_GYRO_CONFIG0, 1, &data);
    this->writeByte(ICM40627_ADDRESS, ICM40627_GYRO_CONFIG0, data);
    

	// status |= inv_icm406xx_read_reg(ICM40627_ADDRESS, ICM40627_GYRO_CONFIG1, 1, &data);
    data = this->readByte(ICM40627_ADDRESS, ICM40627_GYRO_CONFIG1);
	data &= ~BIT_GYRO_CONFIG1_GYRO_UI_FILT_ORD_MASK;
	data |= ST_GYRO_UI_FILT_ORD_IND;
	// status |= inv_icm406xx_write_reg(ICM40627_ADDRESS, ICM40627_GYRO_CONFIG1, 1, &data);
    this->writeByte(ICM40627_ADDRESS, ICM40627_GYRO_CONFIG1, data);

	
	// status |= inv_icm406xx_read_reg(ICM40627_ADDRESS, ICM40627_GYRO_ACCEL_CONFIG0, 1, &data);
    data = this->readByte(ICM40627_ADDRESS, ICM40627_GYRO_ACCEL_CONFIG0);
	data &= ~BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_MASK; 
	data |= ST_GYRO_UI_FILT_BW_IND;
	// status |= inv_icm406xx_write_reg(ICM40627_ADDRESS, ICM40627_GYRO_ACCEL_CONFIG0, 1, &data);
    this->writeByte(ICM40627_ADDRESS, ICM40627_GYRO_ACCEL_CONFIG0, data);


	/* Read average gyro digital output for each axis and store them as STG_OFF_{x,y,z} in lsb */
	status |= average_sensor_output(INV_ICM406XX_SENSOR_ON_MASK_GYRO, 0, STG_OFF);

	/* Enable self-test for each axis and read average gyro digital output 
	 * for each axis and store them as STG_ON_{x,y,z} in lsb */
	status |= average_sensor_output(INV_ICM406XX_SENSOR_ON_MASK_GYRO, 
		(BIT_GYRO_X_ST_EN + BIT_GYRO_Y_ST_EN + BIT_GYRO_Z_ST_EN), STG_ON);

	/* calculate the self-test response as ABS(ST_ON_{x,y,z} - ST_OFF_{x,y,z}) for each axis */
	for(i = 0; i < 3; i++){
		STG_response[i] = INV_ABS(STG_ON[i] - STG_OFF[i]);}


	/* calculate ST results OTP based on the equation */
	bank = 1;
	// status |= inv_icm406xx_write_reg(s, ICM40627_REG_BANK_SEL, 1, &bank);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, bank);

	// status |= inv_icm406xx_read_reg(s, ICM40627_XG_ST_DATA, 3, ST_code_regs);
    this->readBytes(ICM40627_ADDRESS, ICM40627_XG_ST_DATA, 3, ST_code_regs);

	bank = 0;
	// status |= inv_icm406xx_write_reg(s, ICM40627_REG_BANK_SEL, 1, &bank);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, bank);


	for (i = 0; i < 3; i++) {
		int fs_sel = ST_GYRO_FSR >> BIT_GYRO_CONFIG0_FS_SEL_POS;
		STG_OTP[i] = INV_ST_OTP_EQUATION(fs_sel, ST_code_regs[i]);
	}
	
	/** Check Gyro self-test results */
	gyro_sensitivity_1dps = 32768 / reg_to_gyro_fsr(ST_GYRO_FSR);

    // The self-test response is defined as follows:
    // SELF-TEST RESPONSE = SENSOR OUTPUT WITH SELF-TEST ENABLED – SENSOR OUTPUT WITH SELF-TEST DISABLED
    // When the value of the self-test response is within the specified min/max limits of the product specification, the
    // part has passed self-test. When the self-test response exceeds the min/max values, the part is deemed to have
    // failed self-test.

	for (i = 0; i < 3; i++) {
		if (STG_OTP[i]) {
			float ratio = ((float)STG_response[i]) / ((float)STG_OTP[i]);
			if (ratio <= MIN_RATIO_GYRO){
                return FAILED; 
            }
		} else if(STG_response[i] < (MIN_ST_GYRO_DPS * gyro_sensitivity_1dps)) {
            return FAILED; 

		}
	}
    return PASSED;

}
/**
 * @brief Method to self test accelerometer
 * 
 * @return 0 if self test passed 
 * @return 1 if self test failed 
 */
bool ICM40627::runAccelSelfTest()
{
	int status = 0;
	uint8_t data;
	uint8_t bank;

	int32_t STA_OFF[3], STA_ON[3];
	uint32_t STA_response[3];

	uint8_t ST_code_regs[3];
	uint32_t STA_OTP[3];

	int i = 0;
	int axis, axis_sign;
	
	uint32_t accel_sensitivity_1g, gravity; 

	
	/* Set accel configuration */
	// status |= inv_icm406xx_read_reg(s, ICM40627_ACCEL_CONFIG0, 1, &data);
    data = this->readByte(ICM40627_ADDRESS, ICM40627_ACCEL_CONFIG0);
	data &= ~BIT_ACCEL_CONFIG0_FS_SEL_MASK;
	data &= ~BIT_ACCEL_CONFIG0_ODR_MASK;
	data |= ST_ACCEL_FSR;
	data |= ST_ACCEL_ODR;
	// status |= inv_icm406xx_write_reg(s, ICM40627_ACCEL_CONFIG0, 1, &data);
    this->writeByte(ICM40627_ADDRESS, ICM40627_ACCEL_CONFIG0, data);
	
	// status |= inv_icm406xx_read_reg(s, ICM40627_ACCEL_CONFIG1, 1, &data);
    data = this->readByte(ICM40627_ADDRESS, ICM40627_ACCEL_CONFIG1);
	data &= ~BIT_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_MASK;
	data |= ST_ACCEL_UI_FILT_ORD_IND;
	// status |= inv_icm406xx_write_reg(s, ICM40627_ACCEL_CONFIG1, 1, &data);
    this->writeByte(ICM40627_ADDRESS, ICM40627_ACCEL_CONFIG1, data);
	

	// status |= inv_icm406xx_read_reg(s, ICM40627_GYRO_ACCEL_CONFIG0, 1, &data);
    data = this->readByte(ICM40627_ADDRESS, ICM40627_GYRO_ACCEL_CONFIG0);
	data &= ~BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK;
	data |= ST_ACCEL_UI_FILT_BW_IND;
	// status |= inv_icm406xx_write_reg(s, ICM40627_GYRO_ACCEL_CONFIG0, 1, &data);
    this->writeByte(ICM40627_ADDRESS, ICM40627_GYRO_ACCEL_CONFIG0, data);


	/* read average accel digital output for each axis and store them as ST_OFF_{x,y,z} in lsb x 1000 */
	status |= average_sensor_output(INV_ICM406XX_SENSOR_ON_MASK_ACCEL, 0, STA_OFF);

	/* Enable self-test for each axis and read average gyro digital output 
	 * for each axis and store them as ST_ON_{x,y,z} in lsb x 1000 */
	status |= average_sensor_output(INV_ICM406XX_SENSOR_ON_MASK_ACCEL, 
		(BIT_ACCEL_X_ST_EN + BIT_ACCEL_Y_ST_EN + BIT_ACCEL_Z_ST_EN + BIT_ST_REGULATOR_EN), STA_ON);

	/* calculate the self-test response as ABS(ST_ON_{x,y,z} - ST_OFF_{x,y,z}) for each axis */
	/* outputs from this routine are in units of lsb and hence are dependent on the full-scale used on the DUT */
	for(i = 0; i < 3; i++){
		STA_response[i] = INV_ABS(STA_ON[i] - STA_OFF[i]);
        }


	/* calculate ST results OTP based on the equation */
	bank = 2;
	// status |= inv_icm406xx_write_reg(s, ICM40627_REG_BANK_SEL, 1, &bank);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, bank);
	// status |= inv_icm406xx_read_reg(s, ICM40627_XA_ST_DATA, 3, ST_code_regs);
    this->readBytes(ICM40627_ADDRESS, ICM40627_XA_ST_DATA, 3, ST_code_regs);
	bank = 0;
	// status |= inv_icm406xx_write_reg(s, ICM40627_REG_BANK_SEL, 1, &bank);
    this->writeByte(ICM40627_ADDRESS, ICM40627_REG_BANK_SEL, bank);


	for (i = 0; i < 3; i++) {
		int fs_sel = ST_ACCEL_FSR >> BIT_ACCEL_CONFIG0_FS_SEL_POS;
		STA_OTP[i] = INV_ST_OTP_EQUATION(fs_sel, ST_code_regs[i]);
	}
	

	/** Check Accel self-test result */
	accel_sensitivity_1g = 32768 / reg_to_accel_fsr(ST_ACCEL_FSR);

	for (i = 0; i < 3; i++) {
		if (STA_OTP[i]) {
			float ratio = ((float)STA_response[i]) / ((float)STA_OTP[i]);
			if ((ratio >= MAX_RATIO_GYRO) || (ratio <= MIN_RATIO_GYRO)){
                return FAILED;
            }

		} else if ((STA_response[i] < ((MIN_ST_ACCEL_MG * accel_sensitivity_1g) / 1000))
				|| (STA_response[i] > ((MAX_ST_ACCEL_MG * accel_sensitivity_1g) / 1000))) {
                return FAILED;
		}
	}

	/* assume the largest data axis shows +1 or -1 gee for gravity */
	axis = 0;
	axis_sign = 1;
	if (INV_ABS(STA_OFF[1]) > INV_ABS(STA_OFF[0]))
		axis = 1;
	if (INV_ABS(STA_OFF[2]) > INV_ABS(STA_OFF[axis]))
		axis = 2;
	if (STA_OFF[axis] < 0)
		axis_sign = -1;

	gravity = accel_sensitivity_1g * axis_sign;
	STA_OFF[axis] -= gravity;

	return PASSED;
}


int ICM40627::reg_to_accel_fsr(ICM406XX_ACCEL_CONFIG0_FS_SEL_t reg)
{
	switch(reg) {
	case ICM406XX_ACCEL_CONFIG0_FS_SEL_2g:   return 2;
	case ICM406XX_ACCEL_CONFIG0_FS_SEL_4g:   return 4;
	case ICM406XX_ACCEL_CONFIG0_FS_SEL_8g:   return 8;
	case ICM406XX_ACCEL_CONFIG0_FS_SEL_16g:  return 16;
	default:                                 return -1;
	}
}
int ICM40627::reg_to_gyro_fsr(ICM406XX_GYRO_CONFIG0_FS_SEL_t reg)
{
	switch(reg) {
	case ICM406XX_GYRO_CONFIG0_FS_SEL_16dps:   return 16;
	case ICM406XX_GYRO_CONFIG0_FS_SEL_31dps:   return 31;
	case ICM406XX_GYRO_CONFIG0_FS_SEL_62dps:   return 62;
	case ICM406XX_GYRO_CONFIG0_FS_SEL_125dps:  return 125;
	case ICM406XX_GYRO_CONFIG0_FS_SEL_250dps:  return 250;
	case ICM406XX_GYRO_CONFIG0_FS_SEL_500dps:  return 500;
	case ICM406XX_GYRO_CONFIG0_FS_SEL_1000dps: return 1000;
	case ICM406XX_GYRO_CONFIG0_FS_SEL_2000dps: return 2000;
	default:                                   return -1;
	}
}
int ICM40627::average_sensor_output(int sensor, int self_test_config, int32_t average[3])
{
	int status = 0;
	int it = 0; /* Number of sample read */
	int sample_discarded = 0; /* Number of sample discarded */ 
	int timeout = 300; /* us */
	uint8_t data_reg; /* address of the register where to read the data */
	uint8_t self_test_config_reg; /* SELF_TEST_CONFIG register content */
	uint8_t pwr_mgmt_reg; /* PWR_MGMT register content */
	int32_t sum[3] = {0}; /* sum of all data read */

	if(sensor == INV_ICM406XX_SENSOR_ON_MASK_GYRO) {
		data_reg = ICM40627_GYRO_DATA_X1;

		/* Enable Gyro */
		// status |= inv_icm406xx_read_reg(s, ICM40627_PWR_MGMT0, 1, &pwr_mgmt_reg);
        pwr_mgmt_reg = this->readByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0);
		pwr_mgmt_reg &= (uint8_t)~BIT_PWR_MGMT_0_GYRO_MODE_MASK;
		pwr_mgmt_reg |= (uint8_t)ICM406XX_PWR_MGMT_0_GYRO_MODE_LN;
		// status |= inv_icm406xx_write_reg(s, ICM40627_PWR_MGMT0, 1, &pwr_mgmt_reg);
        this->writeByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0, pwr_mgmt_reg);


		/* wait for 60ms to allow output to settle */
		// inv_icm406xx_sleep_us(60*1000);
        delay(60);
	
	} else if(sensor == INV_ICM406XX_SENSOR_ON_MASK_ACCEL) {
		data_reg = ICM40627_ACCEL_DATA_X1;
		
		/* Enable Accel */
		// status |= inv_icm406xx_read_reg(s, ICM40627_PWR_MGMT0, 1, &pwr_mgmt_reg);
        pwr_mgmt_reg = this->readByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0);
		pwr_mgmt_reg &= (uint8_t)~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
		pwr_mgmt_reg |= (uint8_t)ICM406XX_PWR_MGMT_0_ACCEL_MODE_LN;
		// status |= inv_icm406xx_write_reg(s, ICM40627_PWR_MGMT0, 1, &pwr_mgmt_reg);
        this->writeByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0, pwr_mgmt_reg);


		/* wait for 25ms to allow output to settle */
		// inv_icm406xx_sleep_us(25*1000);
        delay(25);
	}
	else
		return INV_ERROR_BAD_ARG; /* Invalid sensor provided */

	/* Apply ST config if required */
	if(self_test_config) {
		// status |= inv_icm406xx_read_reg(s, ICM40627_SELF_TEST_CONFIG, 1, &self_test_config_reg);
        self_test_config_reg = this->readByte(ICM40627_ADDRESS, ICM40627_SELF_TEST_CONFIG);
		self_test_config_reg |= self_test_config; 
		// status |= inv_icm406xx_write_reg(s, ICM40627_SELF_TEST_CONFIG, 1, &self_test_config_reg);
        this->writeByte(ICM40627_ADDRESS, ICM40627_SELF_TEST_CONFIG, self_test_config_reg);


		if(sensor == INV_ICM406XX_SENSOR_ON_MASK_GYRO)
			/* wait 200ms for the oscillation to stabilize */
			// inv_icm406xx_sleep_us(200*1000);
            delay(200);
		else 
			/* wait for 25ms to allow output to settle */
			// inv_icm406xx_sleep_us(25*1000);
            delay(25);
	}

	do {
		uint8_t int_status;
		// status |= inv_icm406xx_read_reg(s, ICM40627_INT_STATUS, 1, &int_status);
        int_status = this->readByte(ICM40627_ADDRESS, ICM40627_INT_STATUS);
		
		if (int_status & BIT_INT_STATUS_DRDY) {
			int16_t sensor_data[3] = {0}; 
			uint8_t sensor_data_reg[6]; /* sensor data registers content */

			/* Read data */
			// status |= inv_icm406xx_read_reg(s, data_reg, 6, sensor_data_reg);
            this->readBytes(ICM40627_ADDRESS, data_reg, 6, sensor_data_reg);

            

			uint8_t intf_config0 =  this->readByte(ICM40627_ADDRESS, ICM40627_INTF_CONFIG0);
            bool endianess_data = (bool)(intf_config0 & ~(BIT_DATA_ENDIAN_POS));
			if (endianess_data){// == ICM406XX_INTF_CONFIG0_DATA_BIG_ENDIAN) {
				sensor_data[0] = (sensor_data_reg[0] << 8) | sensor_data_reg[1];
				sensor_data[1] = (sensor_data_reg[2] << 8) | sensor_data_reg[3];
				sensor_data[2] = (sensor_data_reg[4] << 8) | sensor_data_reg[5];
			} else { // LITTLE ENDIAN
				sensor_data[0] = (sensor_data_reg[1] << 8) | sensor_data_reg[0];
				sensor_data[1] = (sensor_data_reg[3] << 8) | sensor_data_reg[2];
				sensor_data[2] = (sensor_data_reg[5] << 8) | sensor_data_reg[4];
			}
            // Serial.printf("%d,%d,%d\n", sensor_data[0], sensor_data[1], sensor_data[2]);
			if ((sensor_data[0] != -32768) && (sensor_data[1] != -32768) && (sensor_data[2] != -32768)) {
				sum[0] += sensor_data[0];
				sum[1] += sensor_data[1];
				sum[2] += sensor_data[2];
			} else {
				sample_discarded++;
			}
			it++;
		}
		// inv_icm406xx_sleep_us(1000);
        delay(1);
		timeout--;
	} while((it < 200) && (timeout > 0));

	/* Disable Accel and Gyro */
	// status |= inv_icm406xx_read_reg(s, ICM40627_PWR_MGMT0, 1, &pwr_mgmt_reg);
    pwr_mgmt_reg = this->readByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0);
	pwr_mgmt_reg &= (uint8_t)~BIT_PWR_MGMT_0_GYRO_MODE_MASK;
	pwr_mgmt_reg &= (uint8_t)~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
	pwr_mgmt_reg |= (uint8_t)ICM406XX_PWR_MGMT_0_GYRO_MODE_OFF;
	pwr_mgmt_reg |= (uint8_t)ICM406XX_PWR_MGMT_0_ACCEL_MODE_OFF;
	// status |= inv_icm406xx_write_reg(s, ICM40627_PWR_MGMT0, 1, &pwr_mgmt_reg);
    this->writeByte(ICM40627_ADDRESS, ICM40627_PWR_MGMT0, pwr_mgmt_reg);


	/* Disable self-test config if necessary */
	if(self_test_config) {
		self_test_config_reg &= ~self_test_config;
		// status |= inv_icm406xx_write_reg(s, ICM40627_SELF_TEST_CONFIG, 1, &self_test_config_reg);
        this->writeByte(ICM40627_ADDRESS, ICM40627_SELF_TEST_CONFIG, self_test_config_reg);
	}

	/* Compute average value */
	it -= sample_discarded;
	average[0] = (sum[0] / it);
	average[1] = (sum[1] / it);
	average[2] = (sum[2] / it);
	
	return status;
}