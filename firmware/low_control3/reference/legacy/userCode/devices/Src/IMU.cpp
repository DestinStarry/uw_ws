#include "IMU.h"
#include <math.h>

IMU IMU::imu;

void IMU::Init()
{
    /* 失败不卡死系统，仅标记 IMU 不可用 */
    if (BMI088_init() != BMI088_NO_ERROR) {
        return;
    }
    _initialized = true;
#ifdef IMU_USE_MAG
    _mag_available = (ist8310_init() == IST8310_NO_ERROR);
#endif

    /* 加速度计从暂停模式切换到正常模式 */
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
    uint8_t data = (0x7D & 0x7F);
    HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    data = 0x04;
    HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    HAL_Delay(1);
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);

    /* 初始化四元数 */
    pro_data.ins_quat[0] = 1.0f;
    pro_data.ins_quat[1] = 0.0f;
    pro_data.ins_quat[2] = 0.0f;
    pro_data.ins_quat[3] = 0.0f;

    /* 零偏初值（完全对齐 DJI 例程：无加速度计零偏、无陀螺仪零偏校准） */
    raw_data.gyro_offset[0] = 0.0f;
    raw_data.gyro_offset[1] = 0.0f;
    raw_data.gyro_offset[2] = 0.0f;
    raw_data.accel_offset[0] = 0.0f;
    raw_data.accel_offset[1] = 0.0f;
    raw_data.accel_offset[2] = 0.0f;

    /* 初始化完成，提高 SPI 频率（参考 DJI INS_task：PRESCALER_8） */
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
        return;

    dma_state = IMU_DMA_IDLE;
}

void IMU::Handle()
{
    if (!_initialized) return;

    /* 阻塞读取 BMI088 (陀螺仪+加速度计+温度) — 无 DMA，无中断 */
    BMI088_read(raw_data.gyro, raw_data.accel, &raw_data.temp);

    /* 读取磁力计（如可用） */
    if (_mag_available) {
        ist8310_read_mag(raw_data.mag);
    } else {
        raw_data.mag[0] = 0; raw_data.mag[1] = 0; raw_data.mag[2] = 0;
    }

    /* 零偏校正 */
    offset_correction();

    /* IMU→机体 坐标变换（校正 IMU 安装方向） */
    float gyro_b[3], accel_b[3], mag_b[3];
    imu_to_body(pro_data.gyro, gyro_b);
    imu_to_body(pro_data.accel, accel_b);
    imu_to_body(pro_data.mag, mag_b);
    for (int i = 0; i < 3; i++) _gyro_body[i] = gyro_b[i];

    /* 姿态解算（用机体系数据） */
    MahonyAHRSupdate(pro_data.ins_quat,
        gyro_b[0], gyro_b[1], gyro_b[2],
        accel_b[0], accel_b[1], accel_b[2],
        mag_b[0], mag_b[1], mag_b[2]);
    get_angle(pro_data.ins_quat, &pro_data.ins_angle[0], &pro_data.ins_angle[1], &pro_data.ins_angle[2]);
    attitude_update();   /* 输出绝对姿态（参考官方例程） */
}

void IMU::Receive(uint8_t *data, uint16_t len)
{
    (void)data; (void)len;
}

void IMU::trigger_gyro_dma()
{
    if (!(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) &&
        !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)) {
        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)buf.gyro_dma_tx_buf, (uint32_t)buf.gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    }
}

void IMU::trigger_accel_dma()
{
    if (!(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) &&
        !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)) {
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)buf.accel_dma_tx_buf, (uint32_t)buf.accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
    }
}

void IMU::trigger_temp_dma()
{
    if (!(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) &&
        !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)) {
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)buf.accel_temp_dma_tx_buf, (uint32_t)buf.accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
    }
}

void IMU::DMA_IT_Handle()
{
    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx,
            __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET) {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx,
            __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        switch (dma_state) {
        case IMU_DMA_IDLE:
            break;
        case IMU_DMA_READ_GYRO:
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
            BMI088_gyro_read_over(buf.gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, raw_data.gyro);
            dma_state = IMU_DMA_READ_ACCEL;
            trigger_accel_dma();
            break;
        case IMU_DMA_READ_ACCEL:
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
            BMI088_accel_read_over(buf.accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, raw_data.accel, &raw_data.time);
            dma_state = IMU_DMA_READ_TEMP;
            trigger_temp_dma();
            break;
        case IMU_DMA_READ_TEMP:
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
            BMI088_temperature_read_over(buf.accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &raw_data.temp);
            if (_mag_available) ist8310_read_mag(raw_data.mag);
            else { raw_data.mag[0]=0; raw_data.mag[1]=0; raw_data.mag[2]=0; }
            offset_correction();
            MahonyAHRSupdate(pro_data.ins_quat,
                pro_data.gyro[0], pro_data.gyro[1], pro_data.gyro[2],
                pro_data.accel[0], pro_data.accel[1], pro_data.accel[2],
                pro_data.mag[0], pro_data.mag[1], pro_data.mag[2]);
            get_angle(pro_data.ins_quat, &pro_data.ins_angle[0], &pro_data.ins_angle[1], &pro_data.ins_angle[2]);
            attitude_update();
            dma_state = IMU_DMA_IDLE;
            break;
        }
    }
}

void IMU::get_angle(float q[4], float *yaw, float *pitch, float *roll)
{
    *yaw   = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                    2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
                    2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

void IMU::offset_correction()
{
    for (int i = 0; i < 3; i++) {
        pro_data.gyro[i]  = raw_data.gyro[i] - raw_data.gyro_offset[i];
        pro_data.accel[i] = raw_data.accel[i] - raw_data.accel_offset[i];
    }
    /* 参考 DJI 例程：磁力计直接用 ist8310_read_mag 的换算值 (MAG_SEN*temp)，
     * 不做硬铁校正（MAG_OFFSET/SCALE），与官方 INS_task 一致 */
    pro_data.mag[0] = raw_data.mag[0];
    pro_data.mag[1] = raw_data.mag[1];
    pro_data.mag[2] = raw_data.mag[2];
}

/* ===== IMU→机体 坐标变换 =====
 * IMU 芯片安装方向可能与机体坐标系不一致（如旋转 90° / 翻转）。
 * 通过下面的轴映射，把 IMU 的 gyro/accel/mag 旋转到机体坐标系，
 * 这样 Mahony 输出的姿态就是机体姿态。
 *
 * 每行: {目标机体轴} = {源IMU轴, 符号}
 *   0=x, 1=y, 2=z;  符号 +1 同向, -1 反向
 *
 * 默认 = 不旋转（IMU 与机体同向）。按实际安装修改。
 */
static const int8_t IMU_TO_BODY_MAP[3][2] = {
    {0,  1},   /* body_x  <-  +IMU_x */
    {1,  1},   /* body_y  <-  +IMU_y */
    {2,  1},   /* body_z  <-  +IMU_z */
};

void IMU::imu_to_body(const float in[3], float out[3]) const
{
    for (int i = 0; i < 3; i++)
        out[i] = (float)IMU_TO_BODY_MAP[i][1] * in[IMU_TO_BODY_MAP[i][0]];
}

void IMU::attitude_update()
{
    attitude.yaw   = pro_data.ins_angle[0] * 180.0f / PI;
    attitude.pitch = pro_data.ins_angle[1] * 180.0f / PI;
    attitude.roll  = pro_data.ins_angle[2] * 180.0f / PI;

    /* 角速度 (机体系, deg/s) — 用变换后的机体角速度 */
    attitude.roll_v  = _gyro_body[0] * 180.0f / PI;
    attitude.pitch_v = _gyro_body[1] * 180.0f / PI;
    attitude.yaw_v   = _gyro_body[2] * 180.0f / PI;

    /* 四元数 (w,x,y,z) */
    attitude.quat[0] = pro_data.ins_quat[0];
    attitude.quat[1] = pro_data.ins_quat[1];
    attitude.quat[2] = pro_data.ins_quat[2];
    attitude.quat[3] = pro_data.ins_quat[3];
}

void IMU::calibrate_offset()
{
    /* 只校准陀螺仪零偏。
     * 注意：不能把加速度计静止平均值当零偏减掉——
     * 静止时加速度计读数是重力，减掉后 Mahony 会失去俯仰/横滚参考。 */
    float accel[3], gyro[3], temp;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    for (int i = 0; i < 1000; i++) {
        BMI088_read(gyro, accel, &temp);
        gx += gyro[0]; gy += gyro[1]; gz += gyro[2];
        HAL_Delay(1);
    }
    raw_data.gyro_offset[0] = gx / 1000.0f;
    raw_data.gyro_offset[1] = gy / 1000.0f;
    raw_data.gyro_offset[2] = gz / 1000.0f;
}
