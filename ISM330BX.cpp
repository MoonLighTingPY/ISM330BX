#include "ISM330BX.h"
#include <Arduino.h>

// FreeRTOS м'ютекси для синхронізації доступу до I2C та стану сенсора
bool ISM330BXSensor::initRTOS() {
    _i2cMutex = xSemaphoreCreateMutex();
    _stateMutex = xSemaphoreCreateMutex();
    return (_i2cMutex != NULL && _stateMutex != NULL);
}

// Конструктор ISM330BX
ISM330BXSensor::ISM330BXSensor(TwoWire *i2c, uint8_t address) {
    _i2c = i2c;
    _address = address;
}

// Ініціалізація ISM330BX
ISM330BXStatusTypeDef ISM330BXSensor::begin() {
    uint8_t whoAmI = 0;
    int attempts = 0;
    const int maxAttempts = 5;
    
    // Перевіряємо, чи диспетчер FreeRTOS запущений
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
        delay(50);  // Використовуємо звичайну затримку, якщо диспетчер не запущений
    } else {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    #if DEBUG_ENABLE
    Serial.println("Starting ISM330BX initialization...");
    #endif
    
    // Пробуємо зчитати WHO_AM_I реєстр кілька разів (imu може бути не готовий після колд-старту)
    while (attempts < maxAttempts) {
        #if DEBUG_ENABLE
        Serial.print("Attempt ");
        Serial.print(attempts + 1);
        Serial.println(" to read WHO_AM_I...");
        #endif
        
        if (readRegDirect(ISM330BX_WHO_AM_I, &whoAmI) == ISM330BX_STATUS_OK) {
            #if DEBUG_ENABLE
            Serial.print("WHO_AM_I value: 0x");
            Serial.println(whoAmI, HEX);
            #endif
            
            // Успішно зчитали WHO_AM_I, перевіряємо значення
            if (whoAmI == ISM330BX_WHO_AM_I_EXPECTED) {
                break;
            } else {
                #if DEBUG_ENABLE
                Serial.println("WHO_AM_I mismatch, expected 0x71");
                #endif
            }
        } else {
            #if DEBUG_ENABLE
            Serial.println("Failed to read WHO_AM_I");
            #endif
        }
        
        attempts++;
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
            vTaskDelay(pdMS_TO_TICKS(50)); // Чекаємо перед наступною спробою
        } else {
            delay(50); // Фолбек якщо диспетчер все ще не запущений (якого хуя?)
        }
    }
    
    if (attempts >= maxAttempts) {
        #if DEBUG_ENABLE
        Serial.println("Failed to communicate with ISM330BX after multiple attempts");
        #endif
        return ISM330BX_STATUS_ERROR;
    }
    
    // Вмикаємо BDU (Block Data Update) щоб дані не оновлювались під час зчитування
    #if DEBUG_ENABLE
    Serial.println("Setting BDU...");
    #endif
    uint8_t ctrl3 = 0x04; // BDU bit
    if (writeRegDirect(ISM330BX_CTRL3_C, ctrl3) != ISM330BX_STATUS_OK) {
        #if DEBUG_ENABLE
        Serial.println("Failed to set BDU");
        #endif
        return ISM330BX_STATUS_ERROR;
    }
    
    #if DEBUG_ENABLE
    Serial.println("ISM330BX basic initialization successful");
    #endif
    return ISM330BX_STATUS_OK;
}

// Вмикання акселерометра
ISM330BXStatusTypeDef ISM330BXSensor::enableAccelerometer() {
    // Встановлюємо ODR (104 Hz) і FS (±4g)
    uint8_t ctrl1_xl = 0x48; // 0x40 (104Hz) | 0x08 (±4g)
    ISM330BXStatusTypeDef result = writeRegDirect(ISM330BX_CTRL1_XL, ctrl1_xl);
    #if DEBUG_ENABLE
    if (result == ISM330BX_STATUS_OK) {
        Serial.println("Accelerometer enabled successfully");
    } else {
        Serial.println("Failed to enable accelerometer");
    }
    #endif
    return result;
}

// Вмикання гіроскопа
ISM330BXStatusTypeDef ISM330BXSensor::enableGyroscope() {
    uint8_t ctrl2_g = 0x50 | 0x0C; // 0x50 (208Hz) | 0x0C (2000dps)
    
    // Читаємо поточний стан CTRL2_G
    uint8_t current_value = 0;
    if (readRegDirect(ISM330BX_CTRL2_G, &current_value) == ISM330BX_STATUS_OK) {
        #if DEBUG_ENABLE
        Serial.print("Current CTRL2_G value: 0x");
        Serial.println(current_value, HEX);
        #endif
    }
    
    // Ставимо ODR та FS
    ISM330BXStatusTypeDef result = writeRegDirect(ISM330BX_CTRL2_G, ctrl2_g);
    if (result == ISM330BX_STATUS_OK) {
        #if DEBUG_ENABLE
        Serial.println("Gyroscope enabled with higher settings");
        #endif
        
        // Верифікація чи налаштування застосувалися
        uint8_t verify_value = 0;
        if (readRegDirect(ISM330BX_CTRL2_G, &verify_value) == ISM330BX_STATUS_OK) {
            #if DEBUG_ENABLE
            Serial.print("Verified CTRL2_G value: 0x");
            Serial.println(verify_value, HEX);
            if (verify_value != ctrl2_g) {
                Serial.println("WARNING: Gyroscope settings weren't applied correctly!");
            }
            #endif
        }
        
        // Вмикаємо фільтр високих частот для гіроскопа. Мав би помогти від дрейфу при значеннях близьких до нуля
        writeRegDirect(ISM330BX_CTRL7, 0x40);
    } else {
        #if DEBUG_ENABLE
        Serial.println("Failed to enable gyroscope");
        #endif
    }
    return result;
}

// Перевірка готовності даних
bool ISM330BXSensor::checkDataReady() {
    uint8_t status = 0;
    
    if (readRegDirect(ISM330BX_STATUS_REG, &status) != ISM330BX_STATUS_OK) {
        return false;
    }
    
    return (status & 0x03) != 0; // Перевірка XLDA або GDA бітів (біт 0 або 1)
}

// Читання даних з акселерометра
ISM330BXStatusTypeDef ISM330BXSensor::readAcceleration(int32_t *acceleration) {
    uint8_t data[6];
    
    // Читаємо всі 6 регістрів послідовно з високого H до низького L
    ISM330BXStatusTypeDef result = readRegDirect(ISM330BX_OUTZ_L_A, data, 6, pdMS_TO_TICKS(50));
    if (result != ISM330BX_STATUS_OK) {
        return result;
    }
    
    // Комбінуємо всі біти і застосовуємо скейлінг (з даташиту: 0.061 mg/LSB for ±4g range)
    // Порядок: Z, Y, X (даже немцы так не издевались...)
    int16_t rawZ = (int16_t)((data[1] << 8) | data[0]);
    int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
    int16_t rawX = (int16_t)((data[5] << 8) | data[4]);
    
    // Використовуємо фіксовану арифметику для швидшого розрахунку
    // 0.061 mg/LSB = 61/1000, тому множимо на 61 і ділимо на 1000
    static const int32_t SCALE_FACTOR = 61;
    acceleration[0] = (rawX * SCALE_FACTOR) / 1000;
    acceleration[1] = (rawY * SCALE_FACTOR) / 1000;
    acceleration[2] = (rawZ * SCALE_FACTOR) / 1000;
    
    return ISM330BX_STATUS_OK;
}

bool ISM330BXSensor::checkGyroDataReady() {
    uint8_t status = 0;
    
    if (readRegDirect(ISM330BX_STATUS_REG, &status) != ISM330BX_STATUS_OK) {
        return false;
    }
    
    #if DEBUG_ENABLE
    Serial.print("Status register: 0x");
    Serial.println(status, HEX);
    #endif
    
    return (status & 0x02) != 0; // Перевіряємо GDA біт (біт 1)
}

// Читання даних з гіроскопа 
ISM330BXStatusTypeDef ISM330BXSensor::readGyroscope(int32_t *angularRate) {
    uint8_t data[6];
    
    // Читаємо всі 6 регістрів послідовно з високого H до низького L
    ISM330BXStatusTypeDef result = readRegDirect(ISM330BX_OUTX_L_G, data, 6, pdMS_TO_TICKS(50));
    if (result != ISM330BX_STATUS_OK) {
        #if DEBUG_ENABLE
        Serial.println("Failed to read gyroscope data");
        #endif
        return result;
    }
    
    // Об'єднуємо всі біти і застосовуємо скейлінг (з даташиту: 70 mdps/LSB for ±2000 dps range)
    int16_t rawX = (int16_t)((data[1] << 8) | data[0]);
    int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
    int16_t rawZ = (int16_t)((data[5] << 8) | data[4]);
    
    // Знову-ж таки, використовуємо фіксовану арифметику для швидшого розрахунку
    // 70 mdps/LSB for 2000dps range, тому множимо на 70 
    static const int32_t SCALE_FACTOR = 70;
    angularRate[0] = rawX * SCALE_FACTOR;
    angularRate[1] = rawY * SCALE_FACTOR;
    angularRate[2] = rawZ * SCALE_FACTOR;
    
    return ISM330BX_STATUS_OK;
}

// Пряме зчитування реєстру I2C (один байт)
ISM330BXStatusTypeDef ISM330BXSensor::readRegDirect(uint8_t reg, uint8_t *data, TickType_t timeout) {
    // Делегуємо виклик до функції з кількома байтами, просто зчитуючи 1 байт
    return readRegDirect(reg, data, 1, timeout);
}

// Пряме зчитування реєстру I2C(multiple bytes)
ISM330BXStatusTypeDef ISM330BXSensor::readRegDirect(uint8_t reg, uint8_t *data, uint8_t len, TickType_t timeout) {
    // Беремо I2C м'ютекс з таймаутом
    if (xSemaphoreTake(_i2cMutex, timeout) != pdTRUE) {
        #if DEBUG_ENABLE
        Serial.println("[ISM330BX] Failed to obtain I2C mutex");
        #endif
        return ISM330BX_STATUS_ERROR;
    }
    
    uint8_t attempts = 0;
    const uint8_t MAX_ATTEMPTS = 3;
    bool success = false;
    
    // Пробуємо зчитати реєстр кілька разів, з тайм-аутом для RTOS
    while (attempts < MAX_ATTEMPTS && !success) {
        if (attempts > 0) {
            _i2c->end();
            vTaskDelay(pdMS_TO_TICKS(5 * attempts)); // Експоненційна затримка для відновлення
            _i2c->begin();
            _i2c->setClock(400000); // Запевняємося, що тактова частота точно 400kHz
            vTaskDelay(pdMS_TO_TICKS(2)); // Коротенька затримка для стабілізації
        }
        
        _i2c->beginTransmission(_address);
        _i2c->write(reg);
        uint8_t endResult = _i2c->endTransmission(false); // false = без STOP, щоб продовжити передачу
        
        if (endResult != 0) {
            #if DEBUG_ENABLE
            Serial.print("[ISM330BX] I2C address write failed with error ");
            Serial.println(endResult);
            #endif
            attempts++;
            continue;
        }
        
        // Запит байтів з imu
        uint8_t bytesReceived = _i2c->requestFrom(static_cast<uint8_t>(_address), 
                                                static_cast<size_t>(len), 
                                                static_cast<bool>(true)); // true = send STOP
        
        if (bytesReceived != len) {
            #if DEBUG_ENABLE
            Serial.print("[ISM330BX] Requested ");
            Serial.print(len);
            Serial.print(" bytes, but received ");
            Serial.println(bytesReceived);
            #endif
            attempts++;
            continue;
        }
        
        // Читаємо дані з I2C
        for (uint8_t i = 0; i < len; i++) {
            data[i] = _i2c->read();
        }
        
        success = true;
    }
    
    // Завжди звільняємо м'ютекс перед return
    xSemaphoreGive(_i2cMutex);
    
    return success ? ISM330BX_STATUS_OK : ISM330BX_STATUS_ERROR;
}

// Пряме записування в реєстр I2C
ISM330BXStatusTypeDef ISM330BXSensor::writeRegDirect(uint8_t reg, uint8_t data, TickType_t timeout) {
    // Беремо I2C м'ютекс з таймаутом
    if (xSemaphoreTake(_i2cMutex, timeout) != pdTRUE) {
        #if DEBUG_ENABLE
        Serial.println("[ISM330BX] Failed to obtain I2C mutex for write");
        #endif
        return ISM330BX_STATUS_ERROR;
    }
    
    uint8_t attempts = 0;
    const uint8_t MAX_ATTEMPTS = 3;
    bool success = false;
    
    // Спроба запису в реєстр кілька разів
    while (attempts < MAX_ATTEMPTS && !success) {
        if (attempts > 0) {
            _i2c->end();
            vTaskDelay(pdMS_TO_TICKS(5 * attempts)); // Експоненційна затримка для відновлення
            _i2c->begin();
            _i2c->setClock(400000); // Запевняємося, що тактова частота точно 400kHz
            vTaskDelay(pdMS_TO_TICKS(2)); // Маціпунька затримка для стабілізації
        }
        
        _i2c->beginTransmission(_address);
        _i2c->write(reg);
        _i2c->write(data);
        uint8_t endResult = _i2c->endTransmission();
        
        if (endResult != 0) {
            #if DEBUG_ENABLE
            Serial.print("[ISM330BX] I2C write failed with error ");
            Serial.println(endResult);
            #endif
            attempts++;
            continue;
        }
        
        success = true;
    }
    
    // Повертаємо м'ютекс перед return
    xSemaphoreGive(_i2cMutex);
    
    return success ? ISM330BX_STATUS_OK : ISM330BX_STATUS_ERROR;
}

// Вмикання sensor fusion (треба для вектору гравітації)
ISM330BXStatusTypeDef ISM330BXSensor::enableSensorFusion() {
    #if DEBUG_ENABLE
    Serial.println("Enabling sensor fusion for gravity vector...");
    #endif
    
    // Перевіряємо перше значення реєстру EMB_FUNC_EN_A
    uint8_t current_value = 0;
    if (readRegDirect(ISM330BX_EMB_FUNC_EN_A, &current_value) == ISM330BX_STATUS_OK) {
        #if DEBUG_ENABLE
        Serial.print("Current EMB_FUNC_EN_A value: 0x");
        Serial.println(current_value, HEX);
        #endif
    }
    
    // Вмикаємо sensor fusion встановлюючи SFLP_GAME_EN біт
    ISM330BXStatusTypeDef result = writeRegDirect(ISM330BX_EMB_FUNC_EN_A, current_value | ISM330BX_SFLP_GAME_EN);
    
    if (result == ISM330BX_STATUS_OK) {
        #if DEBUG_ENABLE
        Serial.println("Sensor fusion enabled successfully");
        #endif
        
        // Ініціалізуємо sensor fusion
        result = writeRegDirect(ISM330BX_EMB_FUNC_INIT_A, ISM330BX_SFLP_GAME_INIT);
        if (result == ISM330BX_STATUS_OK) {
            #if DEBUG_ENABLE
            Serial.println("Sensor fusion initialized successfully");
            #endif
        } else {
            #if DEBUG_ENABLE
            Serial.println("Failed to initialize sensor fusion");
            #endif
        }
    } else {
        #if DEBUG_ENABLE
        Serial.println("Failed to enable sensor fusion");
        #endif
    }
    
    return result;
}

// Перевірка готовності даних гравітаційного вектора
bool ISM330BXSensor::checkGravityDataReady() {
    // Можна використовувати той же реєстр STATUS_REG, що й для акселерометра
    uint8_t status = 0;
    
    if (readRegDirect(ISM330BX_STATUS_REG, &status) != ISM330BX_STATUS_OK) {
        return false;
    }
    
    // XLDA біт (біт 0) індикує, що дані готові
    // Так як вектор гравітації - це частина даних акселерометра, то перевіряємо його
    return (status & 0x01) != 0;
}

// Читання кватерніонів з IMU (ротаційний вектор)
ISM330BXStatusTypeDef ISM330BXSensor::readQuaternion(float *quat) {
    uint8_t data[6];
    auto result = readRegDirect(ISM330BX_FIFO_DATA_OUT_TAG, data, 6, pdMS_TO_TICKS(50));
    if (result != ISM330BX_STATUS_OK) {
        #if DEBUG_ENABLE
        Serial.println("Failed to read quaternion data");
        #endif
        return result;
    }
    uint16_t *rawQuat = (uint16_t*)data;
    
    // Конвертуємо з half-precision кватерніонів в float
    // Просту конвертація без використання бібліотек.
    // Математику найшов у інеті, + копайлот. Я НЕ ЗНАЮ як воно працює. Це вже ВСЕВИШНЯ математика 
    float sumsq = 0;
    
    for (int i = 0; i < 3; i++) {
        // Simple half-precision to float conversion
        uint16_t h = rawQuat[i];
        uint16_t h_exp = (h & 0x7c00u);
        uint32_t f_sgn = ((uint32_t)h & 0x8000u) << 16;
        uint32_t result = 0;
        
        if (h_exp == 0x0000u) { // Zero or subnormal
            uint16_t h_sig = (h & 0x03ffu);
            if (h_sig == 0) {
                result = f_sgn;
            } else {
                h_sig <<= 1;
                while ((h_sig & 0x0400u) == 0) {
                    h_sig <<= 1;
                    h_exp++;
                }
                uint32_t f_exp = ((uint32_t)(127 - 15 - h_exp)) << 23;
                uint32_t f_sig = ((uint32_t)(h_sig & 0x03ffu)) << 13;
                result = f_sgn + f_exp + f_sig;
            }
        } else if (h_exp == 0x7c00u) { // Infinity or NaN
            result = f_sgn + 0x7f800000u + (((uint32_t)(h & 0x03ffu)) << 13);
        } else { // Normalized
            result = f_sgn + (((uint32_t)(h & 0x7fffu) + 0x1c000u) << 13);
        }
        
        // Convert the bit pattern to float
        float* resultFloat = (float*)&result;
        quat[i] = *resultFloat;
        sumsq += quat[i] * quat[i];
    }
    
    // Calculate the fourth component
    if (sumsq > 1.0f) {
        // Normalize if sum of squares exceeds 1
        float n = sqrt(sumsq);
        quat[0] /= n;
        quat[1] /= n;
        quat[2] /= n;
        sumsq = 1.0f;
    }
    
    // Calculate w component
    quat[3] = sqrt(1.0f - sumsq);
    
    return ISM330BX_STATUS_OK;
}

// Встановлюємо поточний гравітаційний вектор як референс (нуль), використовуючи обчислені кватерніони
bool ISM330BXSensor::setGravityReference() {
    // Берем м м'ютекс стану з таймаутом щоб не зависнути
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        #if DEBUG_ENABLE
        Serial.println("[ISM330BX] Failed to obtain state mutex for setGravityReference");
        #endif
        return false;
    }
    
    int32_t rawGravityVector[3];
    
    // Читаємо теперішній гравітаційний вектор
    if (readRawGravityVector(rawGravityVector) != ISM330BX_STATUS_OK) {
        #if DEBUG_ENABLE
        Serial.println("Failed to read gravity vector for reference");
        #endif
        xSemaphoreGive(_stateMutex);
        return false;
    }
    
    #if DEBUG_ENABLE
    Serial.print("Current raw gravity before reset: X=");
    Serial.print(rawGravityVector[0]);
    Serial.print(", Y=");
    Serial.print(rawGravityVector[1]);
    Serial.print(", Z=");
    Serial.println(rawGravityVector[2]);
    #endif
    
    // Обчислюємо нормаль вектора
    float currentNorm = sqrt(rawGravityVector[0]*rawGravityVector[0] + 
                            rawGravityVector[1]*rawGravityVector[1] + 
                            rawGravityVector[2]*rawGravityVector[2]);
    
    if (currentNorm < 100.0f) { // Передбачаємо ділення на 0 або дуже малий вектор
        #if DEBUG_ENABLE
        Serial.println("Gravity vector magnitude too small");
        #endif
        xSemaphoreGive(_stateMutex);
        return false;
    }
    
    // Напрям, у який хочемо вирівняти вектор гравітації (в даному випадку +Z)
    float targetX = 0.0f;
    float targetY = 0.0f;
    float targetZ = 1.0f; // +Z
    
    // Нормалізовуємо теперішній вектор гравітації
    float gx = rawGravityVector[0] / currentNorm;
    float gy = rawGravityVector[1] / currentNorm;
    float gz = rawGravityVector[2] / currentNorm;
    
    // Перевіряємо, чи обчислений вектор вже близький до цільового
    float dotProduct = gx * targetX + gy * targetY + gz * targetZ;
    if (dotProduct > 0.999f) {
        // Вже вирівняно з цільовим вектором, використовуємо одиничний кватерніон
        _referenceQuat[0] = 0.0f;
        _referenceQuat[1] = 0.0f;
        _referenceQuat[2] = 0.0f;
        _referenceQuat[3] = 1.0f;
        _hasReferenceQuat = true;
        
        #if DEBUG_ENABLE
        Serial.println("Already aligned with target, using identity quaternion");
        #endif
        
        xSemaphoreGive(_stateMutex);
        return true;
    }
    
    // Для випадку, якщо вектори антипаралельні (180°)
    if (dotProduct < -0.999f) {
        // Повертаємо 180 градусів навколо X осі
        _referenceQuat[0] = 1.0f;
        _referenceQuat[1] = 0.0f;
        _referenceQuat[2] = 0.0f;
        _referenceQuat[3] = 0.0f;
        _hasReferenceQuat = true;
        
        #if DEBUG_ENABLE
        Serial.println("Anti-parallel to target, using 180° X-axis rotation");
        #endif
        
        xSemaphoreGive(_stateMutex);
        return true;
    }
    
    // Обчислюємо орієнтовану вісь обертання (вектор перпендикулярний до обох векторів)
    float crossX = gy * targetZ - gz * targetY;
    float crossY = gz * targetX - gx * targetZ;
    float crossZ = gx * targetY - gy * targetX;
    
    // Нормалізвуємо осі обертання
    float crossMag = sqrt(crossX*crossX + crossY*crossY + crossZ*crossZ);
    if (crossMag < 0.001f) {
        #if DEBUG_ENABLE
        Serial.println("Cross product too small, can't determine rotation axis");
        #endif
        xSemaphoreGive(_stateMutex);
        return false;
    }
    
    crossX /= crossMag;
    crossY /= crossMag;
    crossZ /= crossMag;
    
    // Обчислюємо кут і застосовуємо формулу для обчислення кватерніона
    float angle = acos(constrain(dotProduct, -1.0f, 1.0f));
    float halfAngle = angle * 0.5f;
    float sinHalfAngle = sin(halfAngle);
    
    _referenceQuat[0] = crossX * sinHalfAngle;
    _referenceQuat[1] = crossY * sinHalfAngle;
    _referenceQuat[2] = crossZ * sinHalfAngle;
    _referenceQuat[3] = cos(halfAngle);
    
    // Запевнюємося, що кватерніон нормалізований
    float qNorm = sqrt(_referenceQuat[0]*_referenceQuat[0] + 
                      _referenceQuat[1]*_referenceQuat[1] + 
                      _referenceQuat[2]*_referenceQuat[2] + 
                      _referenceQuat[3]*_referenceQuat[3]);
    
    _referenceQuat[0] /= qNorm;
    _referenceQuat[1] /= qNorm;
    _referenceQuat[2] /= qNorm;
    _referenceQuat[3] /= qNorm;
    
    #if DEBUG_ENABLE
    Serial.print("Quaternion: X=");
    Serial.print(_referenceQuat[0], 6);
    Serial.print(", Y=");
    Serial.print(_referenceQuat[1], 6);
    Serial.print(", Z=");
    Serial.print(_referenceQuat[2], 6);
    Serial.print(", W=");
    Serial.println(_referenceQuat[3], 6);
    
    Serial.println("Gravity reference set successfully");
    #endif
    
    _hasReferenceQuat = true;
    xSemaphoreGive(_stateMutex);
    return true;
}

// Застосування референсного (нульового) вектора гравітації до вхідного вектора
bool ISM330BXSensor::applyGravityReference(int32_t *vec) {
    // М'ютекс стану для синхронізації доступу до _referenceQuat
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        #if DEBUG_ENABLE
        Serial.println("[ISM330BX] Failed to obtain state mutex for applyGravityReference");
        #endif
        return false;
    }
    
    if (!_hasReferenceQuat) {
        xSemaphoreGive(_stateMutex);
        return false;
    }
    
    // Переводимо вектор в float для обчислень
    float vecFloat[3];
    for (int i = 0; i < 3; i++) {
        vecFloat[i] = (float)vec[i];
    }
    
    // Застосовуємо обчислений кватерніон для обертання вектора
    float vecRotated[3];
    quaternionRotate(_referenceQuat, vecFloat, vecRotated);
    
    // Конвертуємо назад в int32_t
    for (int i = 0; i < 3; i++) {
        vec[i] = (int32_t)round(vecRotated[i]);
    }
    
    xSemaphoreGive(_stateMutex);
    return true;
}

// Обчислення оберненого кватерніона
void ISM330BXSensor::quaternionInverse(const float *q, float *qInv) {
    // Для обчислення оберненого кватерніона використовуємо формулу q^(-1) = (q*) / |q|^2
    float norm = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    
    if (fabs(norm - 1.0f) > 0.01f) {
        // Не є одиничним кватерніоном, тому нормалізуємо
        float invNorm = 1.0f / norm;
        qInv[0] = -q[0] * invNorm;
        qInv[1] = -q[1] * invNorm;
        qInv[2] = -q[2] * invNorm;
        qInv[3] = q[3] * invNorm;
    } else {
        // Вже одиничний кватерніон, просто обертаємо знак векторної частини
        qInv[0] = -q[0];
        qInv[1] = -q[1];
        qInv[2] = -q[2];
        qInv[3] = q[3];
    }
}

// Ротація кватерніона
void ISM330BXSensor::quaternionRotate(const float *q, const float *v, float *vOut) {
    
    //  q * v * q^(-1)
    float qw = q[3];
    float qx = q[0];
    float qy = q[1];
    float qz = q[2];
    float vx = v[0];
    float vy = v[1];
    float vz = v[2];
    
    float qw2 = qw * qw;
    float qx2 = qx * qx;
    float qy2 = qy * qy;
    float qz2 = qz * qz;
    
    float qwx = qw * qx;
    float qwy = qw * qy;
    float qwz = qw * qz;
    float qxy = qx * qy;
    float qxz = qx * qz;
    float qyz = qy * qz;
    
    vOut[0] = vx * (qw2 + qx2 - qy2 - qz2) + 
              vy * 2 * (qxy - qwz) + 
              vz * 2 * (qxz + qwy);
              
    vOut[1] = vx * 2 * (qxy + qwz) + 
              vy * (qw2 - qx2 + qy2 - qz2) + 
              vz * 2 * (qyz - qwx);
              
    vOut[2] = vx * 2 * (qxz - qwy) + 
              vy * 2 * (qyz + qwx) + 
              vz * (qw2 - qx2 - qy2 + qz2);
}

// Читання сирих даних вектора гравітації (без застосування референсу)
ISM330BXStatusTypeDef ISM330BXSensor::readRawGravityVector(int32_t *gravityVector) {
    uint8_t data[6];
    auto result = readRegDirect(ISM330BX_UI_OUTZ_L_A_DualC, data, 6, pdMS_TO_TICKS(50));
    if (result != ISM330BX_STATUS_OK) {
        #if DEBUG_ENABLE
        Serial.println("Failed to read raw gravity vector data");
        #endif
        return result;
    }
    
    int16_t rawZ = (int16_t)((data[1] << 8) | data[0]);
    int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
    int16_t rawX = (int16_t)((data[5] << 8) | data[4]);
    
    // Використовуємо фіксовану арифметику для швидшого розрахунку
    // 0.488 mg per LSB, тому множимо на 488 і ділимо на 1000.
    // Додатково додаємо 500 для округлення до найближчого цілого.
    // Без цього округлення можуть бути проблеми з точністю. Допустимо x = 1.6,
    // Вийде, що 1.6 * 488 = 780.8, а при діленні на 1000 отримаємо 0.78, що округлиться до 1, а не до 2.
    static constexpr int32_t SCALE_MG_PER_LSB = 488;
    gravityVector[0] = (rawX * SCALE_MG_PER_LSB + 500) / 1000;
    gravityVector[1] = (rawY * SCALE_MG_PER_LSB + 500) / 1000;
    gravityVector[2] = (rawZ * SCALE_MG_PER_LSB + 500) / 1000;
    
    return ISM330BX_STATUS_OK;
}

// Читання вектора гравітації з IMU (вже з встановленим референсом)
ISM330BXStatusTypeDef ISM330BXSensor::readGravityVector(int32_t *gravityVector) {
    uint8_t data[6];
    auto result = readRegDirect(ISM330BX_UI_OUTZ_L_A_DualC, data, 6, pdMS_TO_TICKS(50));
    if (result != ISM330BX_STATUS_OK) {
        #if DEBUG_ENABLE
        Serial.println("Failed to read gravity vector data");
        #endif
        return result;
    }
    
    int16_t rawZ = (int16_t)((data[1] << 8) | data[0]);
    int16_t rawY = (int16_t)((data[3] << 8) | data[2]);
    int16_t rawX = (int16_t)((data[5] << 8) | data[4]);
    
    // Так само, як і в readRawGravityVector, використовуємо фіксовану арифметику, з округленням
    static constexpr int32_t SCALE_MG_PER_LSB = 488;
    gravityVector[0] = (rawX * SCALE_MG_PER_LSB + 500) / 1000;
    gravityVector[1] = (rawY * SCALE_MG_PER_LSB + 500) / 1000;
    gravityVector[2] = (rawZ * SCALE_MG_PER_LSB + 500) / 1000;
    
    // М'ютекс стану для синхронізації доступу до _referenceQuat
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Застосовуємо референсний кватерніон, якщо він встановлений
        if (_hasReferenceQuat) {
            // Зберігаємо оригінальну величину вектора
            float originalMag = sqrt(gravityVector[0]*gravityVector[0] + 
                                    gravityVector[1]*gravityVector[1] + 
                                    gravityVector[2]*gravityVector[2]);
                                    
            // Застосовуємо ротацію
            float vecFloat[3] = {(float)gravityVector[0], (float)gravityVector[1], (float)gravityVector[2]};
            float vecRotated[3];
            quaternionRotate(_referenceQuat, vecFloat, vecRotated);
            
            // Зберігаємо оригінальну величину ротаційного вектора
            float rotatedMag = sqrt(vecRotated[0]*vecRotated[0] + 
                                   vecRotated[1]*vecRotated[1] + 
                                   vecRotated[2]*vecRotated[2]);
            
            // Якщо ротаційний вектор не нульовий, нормалізуємо його
            if (rotatedMag > 0.0001f) {
                float scaleFactor = originalMag / rotatedMag;
                for (int i = 0; i < 3; i++) {
                    vecRotated[i] *= scaleFactor;
                    gravityVector[i] = (int32_t)round(vecRotated[i]);
                }
            }
        }
        
        // Застосовуємо фільтрацію ддя вектора гравітації
        filterGravityVector(gravityVector);
        
        // Відпускаємо м'ютекс
        xSemaphoreGive(_stateMutex);
    }
    
    return ISM330BX_STATUS_OK;
}

// Ввімкнення фільтрації вектора гравітації
void ISM330BXSensor::enableGravityFilter(ISM330BXGravityFilterType type) {
    // Беремо м'ютекс стану з таймаутом
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        _gravityFilterType = type;
        _gravityFilterInitialized = false;  // Скидаємо флаг ініціалізації фільтра
        
        #if DEBUG_ENABLE
        Serial.print("Gravity filter enabled: ");
        switch (_gravityFilterType) {
            case GRAVITY_FILTER_THRESHOLD:
                Serial.println("Threshold-only");
                break;
            case GRAVITY_FILTER_LOWPASS:
                Serial.println("Low-pass");
                break;
            case GRAVITY_FILTER_HYBRID:
                Serial.println("Hybrid (threshold + low-pass)");
                break;
            default:
                Serial.println("None");
                break;
        }
        #endif
        
        xSemaphoreGive(_stateMutex);
    }
}

// Вимкнення фільтрації вектора гравітації
void ISM330BXSensor::disableGravityFilter() {
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        _gravityFilterType = GRAVITY_FILTER_NONE;
        #if DEBUG_ENABLE
        Serial.println("Gravity filter disabled");
        #endif
        xSemaphoreGive(_stateMutex);
    }
}

// Конфігурація порогу для фільтрації
void ISM330BXSensor::configureThreshold(uint16_t threshold) {
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Валідуємо поріг
        // У нас дані вимірюються в mg, тому поріг в 3500mg - це 3.5g, а це вже досить багато, враховуючи що поріг
        // Є зазвичай у межах 0-1000mg (за нормальних умов)
        if (threshold > 3500) {
            #if DEBUG_ENABLE
            Serial.println("[WARNING] Threshold too high, limiting to 3500mg");
            #endif
            threshold = 2000;
        }
        
        _spikeThreshold = threshold;
        
        #if DEBUG_ENABLE
        Serial.print("Gravity filter threshold configured: ");
        Serial.print(_spikeThreshold);
        Serial.println(" mg");
        
        // Фідбек, якщо тип фільтра не використовує поріг
        if (_gravityFilterType == GRAVITY_FILTER_LOWPASS) {
            Serial.println("[NOTE] Current filter type (LOWPASS) doesn't use threshold");
        }
        #endif
        
        xSemaphoreGive(_stateMutex);
    }
}

// Конфігурація альфа для фільтрації по low-pass
void ISM330BXSensor::configureAlpha(float alpha) {
    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        // Валідуємо альфа (0.01 - 1.0)
        // Більша альфа = швидша реакція, але менше фільтрування
        // Менша альфа = повільніша реакція, але більше фільтрування
        _filterAlpha = constrain(alpha, 0.01f, 1.0f);
        
        #if DEBUG_ENABLE
        Serial.print("Gravity filter alpha configured: ");
        Serial.println(_filterAlpha, 3);
        
        // Фідбек, якщо тип фільтра не використовує альфа
        if (_gravityFilterType == GRAVITY_FILTER_THRESHOLD) {
            Serial.println("[NOTE] Current filter type (THRESHOLD) doesn't use alpha");
        }
        #endif
        
        xSemaphoreGive(_stateMutex);
    }
}

// Фільтруємо вектор гравітації, враховуючи обраний тип фільтра
bool ISM330BXSensor::filterGravityVector(int32_t *gravityVector) {
    // Ця функція викликається з readGravityVector, тому м'ютекс стану вже взятий,
    // Тому не потрібно його брати ще раз
    
    // Якщо тип фільтру не встановлений, нічого не робимо
    if (_gravityFilterType == GRAVITY_FILTER_NONE) {
        return true;
    }
    
    // Ініціалізуємо фільтр, якщо ще не зробили цього
    if (!_gravityFilterInitialized) {
        memcpy(_lastGravityVector, gravityVector, sizeof(int32_t) * 3);
        _gravityFilterInitialized = true;
        return true;
    }
    
    bool vectorModified = false;
    
    // Застосовуємо фільтрацію вектора гравітації залежно від типу фільтра
    switch (_gravityFilterType) {
        case GRAVITY_FILTER_THRESHOLD: {
            // Простий пороговий філтьтр
            for (int i = 0; i < 3; i++) {
                if (abs(gravityVector[i] - _lastGravityVector[i]) > _spikeThreshold) {
                    // Стрибок - відкидаємо його
                    gravityVector[i] = _lastGravityVector[i];
                    vectorModified = true;
                } else {
                    // Записуємо валідне значення
                    _lastGravityVector[i] = gravityVector[i];
                }
            }
            break;
        }
        
        case GRAVITY_FILTER_LOWPASS: {
            // Low-pass фільтр (низьких частот)
            for (int i = 0; i < 3; i++) {
                // Apply complementary filter: y = (1-α)*y_prev + α*x
                int32_t filtered = _lastGravityVector[i] + (int32_t)(_filterAlpha * (gravityVector[i] - _lastGravityVector[i]));
                _lastGravityVector[i] = filtered;
                gravityVector[i] = filtered;
            }
            vectorModified = true;
            break;
        }
        
        case GRAVITY_FILTER_HYBRID: {
            // Гібридний фільтр (поріг + low-pass)
            for (int i = 0; i < 3; i++) {
                if (abs(gravityVector[i] - _lastGravityVector[i]) > _spikeThreshold) {
                    gravityVector[i] = _lastGravityVector[i];
                    vectorModified = true;
                } else {
                    int32_t filtered = _lastGravityVector[i] + (int32_t)(_filterAlpha * (gravityVector[i] - _lastGravityVector[i]));
                    _lastGravityVector[i] = filtered;
                    gravityVector[i] = filtered;
                    vectorModified = true;
                }
            }
            break;
        }
        
        default:
            break;
    }
    
    return !vectorModified; // Return true if no spikes were detected/filtered
}