#ifndef IMU_H
#define IMU_H

#include "usermain.h"
#include "BMI088driver.h"
#include "ist8310driver.h"
#include "MahonyAHRS.h"
#include "bsp_spi.h"

#define IMU_USE_MAG 1

#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4

#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1u
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2u

#define MAG_SCALE_X 0.889875
#define MAG_SCALE_Y 0.880329
#define MAG_SCALE_Z 1.229795
#define MAG_OFFSET_X 8.418958
#define MAG_OFFSET_Y -21.033503
#define MAG_OFFSET_Z -4.048424

typedef struct {
    float accel[3], gyro[3], temp, mag[3], accel_offset[3], gyro_offset[3], time;
} IMU_Raw_Data_t;

typedef struct {
    float accel[3], gyro[3], temp, mag[3], ins_quat[4], ins_angle[3];
} IMU_Pro_Data_t;

typedef struct {
    float yaw, pitch, roll;         /* 绝对欧拉角 (deg) */
    float yaw_v, pitch_v, roll_v;   /* 角速度 (deg/s, 机体系) */
    float quat[4];                  /* 四元数 (w,x,y,z) */
} IMU_Attitude_t;

typedef enum {
    IMU_DMA_IDLE = 0,
    IMU_DMA_READ_GYRO,
    IMU_DMA_READ_ACCEL,
    IMU_DMA_READ_TEMP
} IMU_DMA_State_e;

typedef struct {
    /* DMA SPI 读命令: 寄存器地址 | 0x80(读) + 0xFF 占位字节 */
    uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT]        = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT]        = {0};
    uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT]      = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT]      = {0};
    uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};
    uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0};
} IMU_DMA_Buf_t;

#ifdef __cplusplus

class IMU : public Device {
public:
    IMU_Raw_Data_t raw_data;
    IMU_Pro_Data_t pro_data;
    IMU_Attitude_t attitude;
    IMU_DMA_State_e dma_state;

    void Init() override;
    void Handle() override;
    void Receive(uint8_t *data, uint16_t len) override;

    void get_angle(float q[4], float *yaw, float *pitch, float *roll);
    void offset_correction();
    void attitude_update();
    void calibrate_offset();

    /** IMU→机体 坐标变换（校正 IMU 安装方向，可配置轴映射） */
    void imu_to_body(const float in[3], float out[3]) const;

    float _gyro_body[3] = {0.0f, 0.0f, 0.0f};  /* 变换后的机体角速度 (deg/s 换算前, rad/s) */

    void trigger_gyro_dma();
    void trigger_accel_dma();
    void trigger_temp_dma();
    void DMA_IT_Handle();

    bool _initialized = false;
    bool _mag_available = false;

    static IMU imu;

private:
    IMU_DMA_Buf_t buf;
};

#endif /* __cplusplus */

#endif
