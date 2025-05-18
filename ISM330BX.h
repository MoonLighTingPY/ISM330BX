#ifndef ISM330BX_SENSOR_H
#define ISM330BX_SENSOR_H

#include <Wire.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

// Регістри ISM330BX 
#define ISM330BX_WHO_AM_I          0x0F  // Інформація про чіп (ID)
#define ISM330BX_CTRL1_XL          0x10  // Регістр контролю акселерометра
#define ISM330BX_CTRL2_G           0x11  // Регістр контролю гіроскопа
#define ISM330BX_CTRL3_C           0x12  
#define ISM330BX_CTRL7             0x16  
#define ISM330BX_CTRL6_G           0x15  // Регістр контролю гіроскопа (для повного діапазону)
#define ISM330BX_CTRL8_XL          0x17  
#define ISM330BX_FUNC_CFG_ACCESS   0x01
#define ISM330BX_STATUS_REG        0x1E  // Статус (для перевірки готовності даних)
#define ISM330BX_OUTX_L_G          0x22  // Гіроскоп X low byte
#define ISM330BX_OUTX_H_G          0x23  // Гіроскоп X high byte
#define ISM330BX_OUTY_L_G          0x24  // Гіроскоп Y low byte
#define ISM330BX_OUTY_H_G          0x25  // Гіроскоп Y high byte
#define ISM330BX_OUTZ_L_G          0x26  // Гіроскоп Z low byte
#define ISM330BX_OUTZ_H_G          0x27  // Гіроскоп Z high byte
#define ISM330BX_OUTZ_L_A          0x28  // Прискорення Z low byte
#define ISM330BX_OUTZ_H_A          0x29  // Прискорення Z high byte
#define ISM330BX_OUTY_L_A          0x2A  // Прискорення Y low byte
#define ISM330BX_OUTY_H_A          0x2B  // Прискорення Y high byte
#define ISM330BX_OUTX_L_A          0x2C  // Прискорення X low byte
#define ISM330BX_OUTX_H_A          0x2D  // Прискорення X high byte

// Регістри виводу гравітаційного вектору (UI_OUTX/Y/Z для DualC mode)
#define ISM330BX_UI_OUTZ_L_A_DualC 0x34  // Гравітаційний вектор Z low byte
#define ISM330BX_UI_OUTZ_H_A_DualC 0x35  // Гравітаційний вектор Z high byte
#define ISM330BX_UI_OUTY_L_A_DualC 0x36  // Гравітаційний вектор Y low byte
#define ISM330BX_UI_OUTY_H_A_DualC 0x37  // Гравітаційний вектор Y high byte
#define ISM330BX_UI_OUTX_L_A_DualC 0x38  // Гравітаційний вектор X low byte
#define ISM330BX_UI_OUTX_H_A_DualC 0x39  // Гравітаційний вектор X high byte


// Регістри для конфігу і виводу FIFO (для кватерніонів) 
#define ISM330BX_FIFO_DATA_OUT_TAG 0x78
#define ISM330BX_FIFO_CTRL1        0x07
#define ISM330BX_FIFO_CTRL2        0x08
#define ISM330BX_FIFO_CTRL3        0x09
#define ISM330BX_FIFO_CTRL4        0x0A
#define ISM330BX_FIFO_STATUS1      0x1B
#define ISM330BX_FIFO_STATUS2      0x1C
#define ISM330BX_FIFO_DATA_OUT_BYTE_0 0x79
#define ISM330BX_FIFO_DATA_OUT_BYTE_1 0x7A
#define ISM330BX_FIFO_DATA_OUT_BYTE_2 0x7B
#define ISM330BX_FIFO_DATA_OUT_BYTE_3 0x7C
#define ISM330BX_FIFO_DATA_OUT_BYTE_4 0x7D
#define ISM330BX_FIFO_DATA_OUT_BYTE_5 0x7E


#define ISM330BX_EMB_FUNC_EN_A     0x04  // Embedded functions регістр ввімкнення A
#define ISM330BX_EMB_FUNC_INIT_A   0x66  // Embedded functions регістр ініціалізації A
#define ISM330BX_EMB_FUNC_FIFO_EN_A 0x44
#define ISM330BX_EMB_FUNC_FIFO_EN_B 0x45

#define ISM330BX_SFLP_GAME_EN      0x01  // Біт 0 регістра EMB_FUNC_EN_A
#define ISM330BX_SFLP_GAME_INIT    0x01  // Біт 0 регістра EMB_FUNC_INIT_A
#define ISM330BX_SFLP_ODR            0x5E   // SFLP low-power рейт виводу (output data rate)
#define ISM330BX_MLC_FIFO_EN           (1 << 0)
#define ISM330BX_STEP_COUNTER_FIFO_EN  (1 << 1)
#define ISM330BX_SFLP_GBIAS_FIFO_EN    (1 << 2)
#define ISM330BX_SFLP_GRAVITY_FIFO_EN  (1 << 3)
#define ISM330BX_SFLP_GAME_FIFO_EN     (1 << 4)
#define ISM330BX_SFLP_GYROSCOPE_BIAS_TAG        0x03
#define ISM330BX_SFLP_GRAVITY_VECTOR_TAG        0x06
#define ISM330BX_SFLP_GAME_ROTATION_VECTOR_TAG  0x13


#define ISM330BX_WHO_AM_I_EXPECTED 0x71

#ifndef DEBUG_ENABLE
#define DEBUG_ENABLE 1
#endif

// Status
typedef enum {
  ISM330BX_STATUS_OK = 0,
  ISM330BX_STATUS_ERROR
} ISM330BXStatusTypeDef;

typedef enum {
  GRAVITY_FILTER_NONE = 0,     // Без фільтра
  GRAVITY_FILTER_THRESHOLD,    // Простий фільтр з порогом
  GRAVITY_FILTER_LOWPASS,      // Фільтр низьких частот
  GRAVITY_FILTER_HYBRID        // Гібридний фільтр (поріг + low-pass)
} ISM330BXGravityFilterType;

class ISM330BXSensor {
  public:
    bool initRTOS();

    ISM330BXSensor(TwoWire *i2c, uint8_t address = 0x6A);

    ISM330BXStatusTypeDef begin();
    ISM330BXStatusTypeDef enableAccelerometer();
    ISM330BXStatusTypeDef enableGyroscope();
    ISM330BXStatusTypeDef enableSensorFusion();

    ISM330BXStatusTypeDef readAcceleration(int32_t *acceleration);
    ISM330BXStatusTypeDef readGyroscope(int32_t *angularRate);
    ISM330BXStatusTypeDef readGravityVector(int32_t *gravityVector);
    ISM330BXStatusTypeDef readRawGravityVector(int32_t *gravityVector);

    bool checkDataReady();
    bool checkGyroDataReady();
    bool checkGravityDataReady();

    bool setGravityReference();
    bool applyGravityReference(int32_t *vec);

    void enableGravityFilter(ISM330BXGravityFilterType type);
    void disableGravityFilter();
    ISM330BXGravityFilterType getGravityFilterType() { return _gravityFilterType; }
    void configureThreshold(uint16_t threshold);
    void configureAlpha(float alpha);


  private:
    SemaphoreHandle_t _i2cMutex;
    SemaphoreHandle_t _stateMutex;
    TwoWire *_i2c;
    uint8_t _address;

    float _referenceQuat[4] = {1, 0, 0, 0};
    bool _hasReferenceQuat = false;

    void quaternionInverse(const float *q, float *qInv);
    void quaternionRotate(const float *q, const float *v, float *vOut);

    ISM330BXStatusTypeDef readQuaternion(float *quat);
    
    ISM330BXStatusTypeDef readRegDirect(uint8_t reg, uint8_t *data, TickType_t timeout = pdMS_TO_TICKS(50)); // 1 byte
    ISM330BXStatusTypeDef readRegDirect(uint8_t reg, uint8_t *data, uint8_t len, TickType_t timeout = pdMS_TO_TICKS(50)); // multiple bytes
    ISM330BXStatusTypeDef writeRegDirect(uint8_t reg, uint8_t data, TickType_t timeout = pdMS_TO_TICKS(50));

    ISM330BXGravityFilterType _gravityFilterType = GRAVITY_FILTER_NONE;
    uint16_t _spikeThreshold = 500;  // Дефолтне значення порогу (в mg)
    float _filterAlpha = 0.6f;       // Дефолтне alpha для фільтра низьких частот (low-pass)
    int32_t _lastGravityVector[3] = {0, 0, 0};
    bool _gravityFilterInitialized = false;

    bool filterGravityVector(int32_t *gravityVector);
};

#endif
