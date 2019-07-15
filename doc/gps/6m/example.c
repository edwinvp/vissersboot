
/*! This class is a helper for NMEAGPS library to be used in combination a 
 *  u-blox GNSS connected to a (Two)Wire interface (aka I2C, DDC) 
 * 
 *  \note If you intend to use the NMEAGPS library for parsing make sure you
 *  configure the NMEA parsing and last sentence. By default u-blox GNSS start
 *  with the RMC and end with the GLL NMEA sentence. This can be done changing
 *  the NMEAGPS_cfg.h file in the NMEAGPS library adjusting the NMEAGPS_PARSE_*
 *  fields and setting the define LAST_SENTENCE_IN_INTERVAL to NMEAGPS::NMEA_GLL.
 *  
 *  \code
 *  NMEAGPS gps;
 *  gps_fix fix;
 *  GpsTwoWire gpsWire;
 *  void setup() {
 *    Wire.begin();
 *    if (!gpsWire.detect(&Wire))
 *      gpsNotDetectedError();
 *  }
 *  void loop() {
 *    while (gps.available(gpsWire)) {
 *      fix = gps.read();
 *      doSomeWork( fix );
 *    } 
 *  }
 *  \endcode
 */
class GpsTwoWire : public Stream
{
  public:
    /*! detects and configures the I2C interface and detect the GNSS device. 
     *  \param   wire     the physical Wire/I2C interface to use. 
     *  \param   speeed   the Wire/I2C clockrate (default: 400000kHz) 
     *  \returns true     if Gps sucessfully opened and detected
     */
    bool detect(TwoWire* wire, int speed = 400000) {
      if (NULL == wire)
        return false;
      _wire = wire;
      _wire->setClock(speed);
      // detect the device by setting address and reading the ACK
      _wire->beginTransmission(I2C_ADDR);
      _nextRead = millis();
      return (0/*success*/ == _wire->endTransmission());
    }
    /*! check if the GNSS has data available for reading. This first check th 
     *  local buffer and if data is not available it check the Wire/I2C interface 
     *  and reads some data to the local buffer. If no data is received the 
     *  function will not check the hardware interface for #DELAY_MS time in 
     *  subsequent calls unless #force is set true. 
     *  \param      force if set true reads ingnoring the timeout feature
     *  \returns    the number of bytes avilable in the local buffer
     */
    int availableStream(bool force) {
      int sz = _wire->available();
      if (0 == sz) {
        int now = millis();
        if (force || (0 <= (now - _nextRead))) {
          _wire->beginTransmission(I2C_ADDR);
          _wire->write(I2C_REG_LENGTH);
          if (0/*success*/ == _wire->endTransmission(false)) {
            // read the length register
            uint8_t highbyte, lowbyte;
            if (2 == _wire->requestFrom(I2C_ADDR, 2)) {
              highbyte = _wire->read();
              lowbyte = _wire->read();
              int len = 256 * (int)highbyte + (int)lowbyte;
              if (0 < len) {
                // limit the data to be read to the TwoWire internal buffer
                len = min(len, MAX_SZ);
                // read the data of the stream
                len = _wire->requestFrom(I2C_ADDR, len);
                sz += len;
              } else {
                // force a delay after no data to reduce load on i2c
                now += DELAY_MS; 
              }
              _nextRead = now;
            }
          }
        }
      }
      return sz;
    }
    /*! write data to the GNSS through the stream register
     *  pass a valid NMEA or UBX frame including framing and cheksum
     *  \param   data     pointer to the frame to send
     *  \param   quantity the number of bytes in the frame
     *  \returns true     if successful or false in case of error
     */
    bool writeStream(const uint8_t *data, size_t quantity)
    { 
      if ((NULL != data) && (0 < quantity)) {
        do {
          _wire->beginTransmission(I2C_ADDR);
          _wire->write(I2C_REG_STREAM);
          int len = min(quantity, MAX_SZ);
          _wire->write(data, len);
          if (0/*success*/ != _wire->endTransmission())
            return false;
          data += len;
          quantity -= len;
        } while (0 < quantity);
      }
      return true;
    }
    // the Stream interface
    /*virtual*/ int available(void) 
      { return availableStream(false); }
    /*virtual*/ int read(void)            
      { return _wire->read(); }
    /*virtual*/ int peek(void)            
      { return _wire->peek(); }
  protected: 
    /*virtual*/ size_t write(uint8_t data) // write should not be used, use writeStream
      { return _wire->write(data); } 
  private: 
    static const int     MAX_SZ   = 32;   //!< max number of bytes to read to local buffer 
    static const int     DELAY_MS = 50;   //!< limit reading the Wire/I2C by this time if no data received
    static const uint8_t I2C_ADDR = 0x42; //!< 7 bit Wire/I2C address for u-blox
    typedef enum { I2C_REG_LENGTH = 0xFD, //!< register offset for a 16 bit length field [HILO]
                   I2C_REG_STREAM = 0xFF  //!< stream of GNSS/NMEA, does not increment registe offset
                 } I2C_REG; //!< Wire/I2C register offsets 
    TwoWire* _wire;         //!< Wire/I2C hardware interface
    int      _nextRead;     //!< time of next possible read 
};