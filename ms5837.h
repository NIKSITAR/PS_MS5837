#ifndef MS5837_H
#define MS5837_H

#include "wiringPi.h"
#include "wiringPiI2C.h"
#include <iostream>
#include <QDebug>
#include <cmath>
#include "linux/i2c.h"

extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}


class MS5837
{
public:
    MS5837();
    static const float Pa;
    static const float bar;
    static const float mbar;

    static const uint8_t MS5837_30BA;
    static const uint8_t MS5837_02BA;
    static const uint8_t MS5837_UNRECOGNISED;
    int devId = 0x76;
    int fd =0;

    bool init();
    bool begin(); // Calls init()

    /** Установите модель датчика MS5837. Допустимыми параметрами являются MS5837::MS5837_30BA (по умолчанию)
     * и MS5837::MS5837_02BA.
     */
    void setModel(uint8_t model);
    uint8_t getModel();

    /** Укажите плотность рабочей жидкости в кг/м^3. По умолчанию используется для
     * морской воды. Для пресной воды должно быть 997.
     */
    void setFluidDensity(float density);

    /** Считывание с I2C занимает до 40 мс, поэтому возможно экономное использование.
     */
    void read();

    /** Возвращаемое давление в мбар или мбар*коэффициент пересчета.
     */
    float pressure(float conversion = 1.0f);

    /** Возвращаемая температура в градусах C.
     */
    float temperature();

    /** Возвращаемая глубина в метрах (допустима для работы в несжимаемой
     * только жидкости. Используется плотность, установленная для пресной или морской воды.
     */
    float depth();

    /** Возвращаемая высота в метрах (действительна только для работы в воздухе).
     */
    float altitude();
    __u16 C[8];
    unsigned char* per;
    __u32 D1_pres, D2_temp;
    int32_t TEMP;
    int32_t P;
    uint8_t _model;

    float fluidDensity = 1029;

    /** Выполняет вычисления в соответствии с техническими данными датчика для преобразования и
    * компенсации второго порядка.
    */
    void calculate();

    uint8_t crc4(uint16_t n_prom[]);
};

#endif // MS5837_H
