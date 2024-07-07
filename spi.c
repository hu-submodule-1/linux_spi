/**
 * @file      : spi.c
 * @brief     : Linux平台SPI驱动源文件
 * @author    : huenrong (huenrong1028@outlook.com)
 * @date      : 2023-01-18 15:10:12
 *
 * @copyright : Copyright (c) 2023 huenrong
 *
 * @history   : date       author          description
 *              2023-01-18 huenrong        创建文件
 *
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>

#include "./spi.h"

// SPI设备名最大长度
#define SPI_DEV_NAME_MAX_LEN 30

// SPI设备最大数量
#define SPI_DEV_MAX_NUM 10

// SPI设备信息结构体
typedef struct
{
    char spi_dev_name[SPI_DEV_NAME_MAX_LEN]; // SPI设备名
    int spi_dev_fd;                          // SPI设备文件描述符
    pthread_mutex_t spi_dev_mutex;           // SPI设备互斥锁
} spi_dev_info_t;

// 已打开的SPI设备数量
static uint8_t g_spi_dev_num = 0;
// SPI设备信息
static spi_dev_info_t g_spi_dev_info[SPI_DEV_MAX_NUM];
// SPI单次最大传输长度
static uint32_t g_spi_max_transfer_len = 0;

/**
 * @brief  查找指定SPI设备的信息
 * @param  spi_dev_info        : 输出参数, 查找到的SPI设备信息
 * @param  spi_dev_name: 输入参数, 待查找的SPI设备名
 * @return true : 成功
 * @return false: 失败
 */
static bool spi_find_dev_info(spi_dev_info_t *spi_dev_info, const char *spi_dev_name)
{
    int ret = -1;

    if ((!spi_dev_info) || (!spi_dev_name))
    {
        return false;
    }

    for (uint8_t i = 0; i < g_spi_dev_num; i++)
    {
        ret = memcmp(g_spi_dev_info[i].spi_dev_name, spi_dev_name, strlen(spi_dev_name));
        if (0 == ret)
        {
            memcpy(spi_dev_info, &g_spi_dev_info[i], sizeof(spi_dev_info_t));

            return true;
        }
    }

    return false;
}

/**
 * @brief  打开SPI设备
 * @param  spi_dev_name: 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @param  spi_mode    : 输入参数, SPI模式
 * @param  spi_speed   : 输入参数, SPI设备默认最大速度(单位: Hz)
 * @param  spi_bits    : 输入参数, SPI设备字长
 * @return true : 成功
 * @return false: 失败
 */
bool spi_open(const char *spi_dev_name, const spi_mode_e spi_mode, const uint32_t spi_speed, const uint8_t spi_bits)
{
    int ret = -1;
    // SPI设备描述符
    int fd = -1;
    // SPI设备信息
    spi_dev_info_t spi_dev_info = {0};

    if (!spi_dev_name)
    {
        return false;
    }

    // 超过支持的SPI设备数量, 直接返回错误
    if (g_spi_dev_num > SPI_DEV_MAX_NUM)
    {
        return false;
    }

    // SPI设备已打开, 先关闭SPI设备
    if (spi_find_dev_info(&spi_dev_info, spi_dev_name))
    {
        // 关闭SPI设备
        spi_close(spi_dev_info.spi_dev_name);
    }

    // 打开SPI设备
    fd = open(spi_dev_name, O_RDWR);
    if (fd < 0)
    {
        return false;
    }

    // 设置SPI写模式
    ret = ioctl(fd, SPI_IOC_WR_MODE, &spi_mode);
    if (ret < 0)
    {
        close(fd);

        return false;
    }

    // 设置SPI读模式
    ret = ioctl(fd, SPI_IOC_RD_MODE, &spi_mode);
    if (ret < 0)
    {
        close(fd);

        return false;
    }

    // 设置SPI写最大速率
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
    if (ret < 0)
    {
        close(fd);

        return false;
    }

    // 设置SPI读最大速率
    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
    if (ret < 0)
    {
        close(fd);

        return false;
    }

    // 设置写SPI字长
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits);
    if (ret < 0)
    {
        close(fd);

        return false;
    }

    // 设置读SPI字长
    ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits);
    if (ret < 0)
    {
        close(fd);

        return false;
    }

    // 记录SPI设备信息
    memcpy(g_spi_dev_info[g_spi_dev_num].spi_dev_name, spi_dev_name, strlen(spi_dev_name));
    g_spi_dev_info[g_spi_dev_num].spi_dev_fd = fd;

    // 初始化互斥锁
    pthread_mutex_init(&g_spi_dev_info[g_spi_dev_num].spi_dev_mutex, NULL);

    g_spi_dev_num++;

    return true;
}

/**
 * @brief  设置SPI单次最大传输长度, 未设置或设置值为0会使用内部默认值SPI_MAX_TRANSFER_LEN
 * @param  max_transfer_len: 输入参数, 单次最大传输长度
 */
void spi_set_max_transfer_len(const uint32_t max_transfer_len)
{
    g_spi_max_transfer_len = max_transfer_len;
}

/**
 * @brief  关闭SPI设备
 * @param  spi_dev_name: 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @return true : 成功
 * @return false: 失败
 */
bool spi_close(const char *spi_dev_name)
{
    int ret = -1;

    if (!spi_dev_name)
    {
        return false;
    }

    for (uint8_t i = 0; i < g_spi_dev_num; i++)
    {
        ret = memcmp(g_spi_dev_info[i].spi_dev_name, spi_dev_name, strlen(spi_dev_name));
        // 当前SPI设备已打开
        if (0 == ret)
        {
            pthread_mutex_lock(&g_spi_dev_info[i].spi_dev_mutex);

            // 关闭SPI设备
            ret = close(g_spi_dev_info[i].spi_dev_fd);
            if (ret < 0)
            {
                return false;
            }

            pthread_mutex_unlock(&g_spi_dev_info[i].spi_dev_mutex);

            // 清空SPI设备信息
            g_spi_dev_info[i].spi_dev_fd = -1;
            memset(g_spi_dev_info[i].spi_dev_name, 0, SPI_DEV_NAME_MAX_LEN);

            // 销毁互斥锁
            pthread_mutex_destroy(&g_spi_dev_info[i].spi_dev_mutex);

            // 将SPI设备信息放到数组最前面
            memcpy(&g_spi_dev_info[i], &g_spi_dev_info[i + 1], (sizeof(spi_dev_info_t) * (SPI_DEV_MAX_NUM - i - 1)));

            (g_spi_dev_num > 0) ? g_spi_dev_num-- : 0;

            return true;
        }
    }

    return true;
}

/**
 * @brief  向无寄存器地址的SPI从设备发送1字节数据
 * @param  spi_dev_name: 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @param  write_data  : 输入参数, 待发送数据
 * @return true : 成功
 * @return false: 失败
 */
bool spi_write_byte(const char *spi_dev_name, const uint8_t write_data)
{
    int ret = -1;
    // 要发送的数据
    uint8_t write_buf[3] = {0};
    struct spi_ioc_transfer spi_transfer = {0};
    // SPI设备信息
    spi_dev_info_t spi_dev_info = {0};

    if (!spi_dev_name)
    {
        return false;
    }

    // SPI设备未打开, 直接返回失败
    if (!spi_find_dev_info(&spi_dev_info, spi_dev_name))
    {
        return false;
    }

    pthread_mutex_lock(&spi_dev_info.spi_dev_mutex);

    // 要写入的数据
    write_buf[0] = write_data;

    // spi传输结构体赋值
    memset(&spi_transfer, 0, sizeof(spi_transfer));
    // 发送的数据
    spi_transfer.tx_buf = (unsigned long)write_buf;
    // 缓冲长度(发送数据长度)
    spi_transfer.len = 1;
    // cs_change结束之后是否需要改变片选线, 一般在用户空间控制
    spi_transfer.cs_change = 0;

    // 发送数据
    ret = ioctl(spi_dev_info.spi_dev_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
    // 发送失败
    if (-1 == ret)
    {
        pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

        return false;
    }

    pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

    return true;
}

/**
 * @brief  向无寄存器地址的SPI从设备发送n字节数据(n > 1)
 * @param  spi_dev_name  : 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @param  write_data    : 输入参数, 待发送数据
 * @param  write_data_len: 输入参数, 待发送数据长度
 * @return true : 成功
 * @return false: 失败
 */
bool spi_write_nbyte(const char *spi_dev_name, const uint8_t *write_data, const uint32_t write_data_len)
{
    int ret = -1;
    // 未发送数据长度
    uint32_t remain_data_len = 0;
    // 本次发送数据长度
    uint32_t current_data_len = 0;
    // 本次发送数据偏移量
    uint32_t data_offset = 0;
    // 累计传输数据长度
    int transferred_data_len = 0;
    struct spi_ioc_transfer spi_transfer = {0};
    // SPI设备信息
    spi_dev_info_t spi_dev_info = {0};
    // 单次最大传输长度
    uint32_t spi_max_transfer_len = 0;

    if (!spi_dev_name)
    {
        return false;
    }

    // SPI设备未打开, 直接返回失败
    if (!spi_find_dev_info(&spi_dev_info, spi_dev_name))
    {
        return false;
    }

    // 未发送数据长度
    remain_data_len = write_data_len;

    spi_max_transfer_len = (g_spi_max_transfer_len == 0) ? SPI_MAX_TRANSFER_LEN : g_spi_max_transfer_len;

    pthread_mutex_lock(&spi_dev_info.spi_dev_mutex);

    for (uint32_t i = 0; remain_data_len > 0; ++i)
    {
        // 计算本次发送数据长度(未发送数据长度和单次最大传输长度比较)
        current_data_len = ((remain_data_len < spi_max_transfer_len) ? remain_data_len : spi_max_transfer_len);

        // 计算本次发送数据偏移量
        data_offset = (i * spi_max_transfer_len);

        // spi传输结构体赋值
        spi_transfer.tx_buf = (unsigned long)(write_data + data_offset);
        spi_transfer.len = current_data_len;
        spi_transfer.cs_change = 0;

        // 发送数据
        ret = ioctl(spi_dev_info.spi_dev_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
        // 发送失败
        if (-1 == ret)
        {
            pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

            return false;
        }
        // 发送成功
        else
        {
            // 计算累计传输数据长度
            transferred_data_len += spi_transfer.len;

            // 计算未发送数据长度
            remain_data_len -= current_data_len;
        }
    }

    // 数据未全部发送
    if (write_data_len != transferred_data_len)
    {
        pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

        return false;
    }

    pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

    return true;
}

/**
 * @brief  从无寄存器地址的SPI从设备读取1字节数据
 * @param  read_data   : 输出参数, 读取到的数据
 * @param  spi_dev_name: 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @return true : 成功
 * @return false: 失败
 */
bool spi_read_byte(uint8_t *read_data, const char *spi_dev_name)
{
    int ret = -1;
    struct spi_ioc_transfer spi_transfer = {0};
    // SPI设备信息
    spi_dev_info_t spi_dev_info = {0};

    if (!spi_dev_name)
    {
        return false;
    }

    // SPI设备未打开, 直接返回失败
    if (!spi_find_dev_info(&spi_dev_info, spi_dev_name))
    {
        return false;
    }

    pthread_mutex_lock(&spi_dev_info.spi_dev_mutex);

    // spi传输结构体赋值
    memset(&spi_transfer, 0, sizeof(spi_transfer));
    spi_transfer.rx_buf = (unsigned long)read_data;
    spi_transfer.len = 1;
    spi_transfer.cs_change = 0;

    ret = ioctl(spi_dev_info.spi_dev_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
    if (-1 == ret)
    {
        pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

        return false;
    }

    pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

    return true;
}

/**
 * @brief  从无寄存器地址的SPI从设备读取n字节数据(n > 1)
 * @param  read_data    : 输出参数, 读取到的数据
 * @param  spi_dev_name : 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @param  read_data_len: 输入参数, 指定读取的长度
 * @return true : 成功
 * @return false: 失败
 */
bool spi_read_nbyte(uint8_t *read_data, const char *spi_dev_name, const uint16_t read_data_len)
{
    int ret = -1;
    // 未读取数据长度
    uint32_t remain_data_len = 0;
    // 本次读取数据长度
    uint32_t current_data_len = 0;
    // 本次读取数据偏移量
    uint32_t data_offset = 0;
    // 累计传输数据长度
    int transferred_data_len = 0;
    struct spi_ioc_transfer spi_transfer;
    // SPI设备信息
    spi_dev_info_t spi_dev_info = {0};
    // 单次最大传输长度
    uint32_t spi_max_transfer_len = 0;

    if ((!spi_dev_name) || (!read_data))
    {
        return false;
    }

    // 长度错误, 直接返回失败
    if (read_data_len <= 1)
    {
        return false;
    }

    // 串口未打开, 直接返回失败
    if (!spi_find_dev_info(&spi_dev_info, spi_dev_name))
    {
        return false;
    }

    pthread_mutex_lock(&spi_dev_info.spi_dev_mutex);

    // 未读取数据长度
    remain_data_len = read_data_len;

    spi_max_transfer_len = (g_spi_max_transfer_len == 0) ? SPI_MAX_TRANSFER_LEN : g_spi_max_transfer_len;

    // 循环读取数据
    for (uint32_t i = 0; remain_data_len > 0; ++i)
    {
        // 计算本次读取数据长度(未发送数据长度和单次最大传输长度比较)
        current_data_len = (remain_data_len < spi_max_transfer_len) ? remain_data_len : spi_max_transfer_len;

        // 计算本次读取数据偏移量
        data_offset = (i * spi_max_transfer_len);

        // spi传输结构体赋值
        spi_transfer.rx_buf = (unsigned long)(read_data + data_offset);
        spi_transfer.len = current_data_len;
        spi_transfer.cs_change = 0;

        // 读取数据
        ret = ioctl(spi_dev_info.spi_dev_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
        // 读取失败
        if (-1 == ret)
        {
            pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

            return false;
        }
        // 读取成功
        else
        {
            // 计算累计传输数据长度
            transferred_data_len += spi_transfer.len;

            // 计算未发送数据长度
            remain_data_len -= current_data_len;
        }
    }

    // 数据未全部读取
    if (transferred_data_len != read_data_len)
    {
        pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

        return false;
    }

    pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

    return true;
}

/**
 * @brief  向有寄存器地址的SPI从设备发送1字节数据
 * @param  spi_dev_name: 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @param  reg_addr    : 输入参数, 寄存器地址
 * @param  write_data  : 输入参数, 待发送数据
 * @return true : 成功
 * @return false: 失败
 */
bool spi_write_byte_sub(const char *spi_dev_name, const uint8_t reg_addr, const uint8_t write_data)
{
    // SPI写数据时, 先发送要写入的寄存器地址, 再发送要写入的数据

    int ret = -1;
    // 要发送的寄存器地址+数据
    uint8_t write_buf[3] = {0};
    struct spi_ioc_transfer spi_transfer = {0};
    // SPI设备信息
    spi_dev_info_t spi_dev_info = {0};

    if (!spi_dev_name)
    {
        return false;
    }

    // SPI设备未打开, 直接返回失败
    if (!spi_find_dev_info(&spi_dev_info, spi_dev_name))
    {
        return false;
    }

    pthread_mutex_lock(&spi_dev_info.spi_dev_mutex);

    // 寄存器地址
    write_buf[0] = reg_addr;
    // 要写入的数据
    write_buf[1] = write_data;

    // spi传输结构体赋值
    memset(&spi_transfer, 0, sizeof(spi_transfer));
    // 发送的寄存器地址+数据
    spi_transfer.tx_buf = (unsigned long)write_buf;
    // 缓冲长度(发送数据长度)
    spi_transfer.len = 2;
    // cs_change结束之后是否需要改变片选线, 一般在用户空间控制
    spi_transfer.cs_change = 0;

    // 发送数据
    ret = ioctl(spi_dev_info.spi_dev_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
    // 发送失败
    if (-1 == ret)
    {
        pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

        return false;
    }

    pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

    return true;
}

/**
 * @brief  向有寄存器地址的SPI从设备发送n字节数据(n > 1)
 * @param  spi_dev_name  : 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @param  reg_addr      : 输入参数, 寄存器地址
 * @param  write_data    : 输入参数, 待发送数据
 * @param  write_data_len: 输入参数, 待发送数据长度
 * @return true : 成功
 * @return false: 失败
 */
bool spi_write_nbyte_sub(const char *spi_dev_name, const uint8_t reg_addr,
                         const uint8_t *write_data, const uint32_t write_data_len)
{
    // SPI写数据时, 先发送要写入的寄存器地址, 再发送要写入的数据

    int ret = -1;
    uint8_t write_buf[2] = {0};
    // 未发送数据长度
    uint32_t remain_data_len = 0;
    // 本次发送数据长度
    uint32_t current_data_len = 0;
    // 本次发送数据偏移量
    uint32_t data_offset = 0;
    // 累计传输数据长度
    int transferred_data_len = 0;
    struct spi_ioc_transfer spi_transfer[2];
    // SPI设备信息
    spi_dev_info_t spi_dev_info = {0};

    if ((!spi_dev_name) || (!write_data))
    {
        return false;
    }

    // 长度错误, 直接返回失败
    if (write_data_len <= 1)
    {
        return false;
    }

    // SPI设备未打开, 直接返回失败
    if (!spi_find_dev_info(&spi_dev_info, spi_dev_name))
    {
        return false;
    }

    pthread_mutex_lock(&spi_dev_info.spi_dev_mutex);

    // 寄存器地址
    write_buf[0] = reg_addr;

    // 未发送数据长度
    remain_data_len = write_data_len;

    // spi_transfer[0]发送要写入的寄存器地址
    memset(&spi_transfer, 0, sizeof(spi_transfer));
    spi_transfer[0].tx_buf = (unsigned long)&write_buf[0];
    spi_transfer[0].len = 1;
    spi_transfer[0].cs_change = 0;

    // 循环发送数据
    // spi_transfer[1]发送数据
    for (uint32_t i = 0; remain_data_len > 0; ++i)
    {
        // 计算本次发送数据长度(未发送数据长度和单次最大传输长度比较)
        current_data_len = ((remain_data_len < SPI_MAX_TRANSFER_LEN) ? remain_data_len : SPI_MAX_TRANSFER_LEN);

        // 计算本次发送数据偏移量
        data_offset = (i * SPI_MAX_TRANSFER_LEN);

        // spi传输结构体赋值
        spi_transfer[1].tx_buf = (unsigned long)(write_data + data_offset);
        spi_transfer[1].len = current_data_len;
        spi_transfer[1].cs_change = 0;

        // 发送数据
        ret = ioctl(spi_dev_info.spi_dev_fd, SPI_IOC_MESSAGE(2), &spi_transfer);
        // 发送失败
        if (-1 == ret)
        {
            pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

            return false;
        }
        // 发送成功
        else
        {
            // 计算累计传输数据长度
            transferred_data_len += (ret - spi_transfer[0].len);

            // 计算未发送数据长度
            remain_data_len -= current_data_len;
        }
    }

    // 数据未全部发送
    if (write_data_len != transferred_data_len)
    {
        pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

        return false;
    }

    pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

    return true;
}

/**
 * @brief  从有寄存器地址的SPI从设备读取1字节数据
 * @param  read_data   : 输出参数, 读取到的数据
 * @param  spi_dev_name: 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @param  reg_addr    : 输入参数, 寄存器地址
 * @return true : 成功
 * @return false: 失败
 */
bool spi_read_byte_sub(uint8_t *read_data, const char *spi_dev_name, const uint8_t reg_addr)
{
    // SPI读数据时, 先发送要读取的寄存器地址, 再读取数据

    int ret = -1;
    uint8_t write_buf[1] = {0};
    struct spi_ioc_transfer spi_transfer[2];
    // SPI设备信息
    spi_dev_info_t spi_dev_info = {0};

    if ((!spi_dev_name) || (!read_data))
    {
        return false;
    }

    // 串口未打开, 直接返回失败
    if (!spi_find_dev_info(&spi_dev_info, spi_dev_name))
    {
        return false;
    }

    pthread_mutex_lock(&spi_dev_info.spi_dev_mutex);

    // 寄存器地址
    write_buf[0] = reg_addr;

    // 半双工模式, 先发送, 再读取
    memset(&spi_transfer, 0, sizeof(spi_transfer));
    // spi_transfer[0]发送要读取的寄存器地址
    spi_transfer[0].tx_buf = (unsigned long)write_buf;
    spi_transfer[0].len = 1;
    spi_transfer[0].cs_change = 0;

    // spi_transfer[1]读取数据
    memset(read_data, 0, 1);
    spi_transfer[1].rx_buf = (unsigned long)read_data;
    // 指定读取长度
    spi_transfer[1].len = 1;
    spi_transfer[1].cs_change = 0;

    ret = ioctl(spi_dev_info.spi_dev_fd, SPI_IOC_MESSAGE(2), &spi_transfer);
    if (-1 == ret)
    {
        pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

        return false;
    }

    pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

    return true;
}

/**
 * @brief  从有寄存器地址的SPI从设备读取n字节数据(n > 1)
 * @param  read_data    : 输出参数, 读取到的数据
 * @param  spi_dev_name : 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @param  reg_addr     : 输入参数, 寄存器地址
 * @param  read_data_len: 输入参数, 指定读取的长度
 * @return true : 成功
 * @return false: 失败
 */
bool spi_read_nbyte_sub(uint8_t *read_data, const char *spi_dev_name,
                        const uint8_t reg_addr, const uint16_t read_data_len)
{
    // SPI读数据时, 先发送要读取的寄存器地址, 再读取数据

    int ret = -1;
    uint8_t write_buf[2] = {0};
    // 未读取数据长度
    uint32_t remain_data_len = 0;
    // 本次读取数据长度
    uint32_t current_data_len = 0;
    // 本次读取数据偏移量
    uint32_t data_offset = 0;
    // 累计传输数据长度
    int transferred_data_len = 0;
    struct spi_ioc_transfer spi_transfer[2];
    // SPI设备信息
    spi_dev_info_t spi_dev_info = {0};

    if ((!spi_dev_name) || (!read_data))
    {
        return false;
    }

    // 长度错误, 直接返回失败
    if (read_data_len <= 1)
    {
        return false;
    }

    // 串口未打开, 直接返回失败
    if (!spi_find_dev_info(&spi_dev_info, spi_dev_name))
    {
        return false;
    }

    pthread_mutex_lock(&spi_dev_info.spi_dev_mutex);

    // 寄存器地址(最高位为0, 表示读数据)
    write_buf[0] = reg_addr;

    // 未读取数据长度
    remain_data_len = read_data_len;

    // 半双工模式, 先发送, 再读取
    memset(&spi_transfer, 0, sizeof(spi_transfer));
    // spi_transfer[0]发送要读取的寄存器地址
    spi_transfer[0].tx_buf = (unsigned long)&write_buf[0];
    spi_transfer[0].len = 1;
    spi_transfer[0].cs_change = 0;

    // 循环读取数据
    // spi_transfer[1]读取数据
    for (uint32_t i = 0; remain_data_len > 0; ++i)
    {
        // 计算本次读取数据长度(未发送数据长度和单次最大传输长度比较)
        current_data_len = (remain_data_len < SPI_MAX_TRANSFER_LEN) ? remain_data_len : SPI_MAX_TRANSFER_LEN;

        // 计算本次读取数据偏移量
        data_offset = (i * SPI_MAX_TRANSFER_LEN);

        // spi传输结构体赋值
        spi_transfer[1].rx_buf = (unsigned long)(read_data + data_offset);
        spi_transfer[1].len = current_data_len;
        spi_transfer[1].cs_change = 0;

        // 读取数据
        ret = ioctl(spi_dev_info.spi_dev_fd, SPI_IOC_MESSAGE(2), &spi_transfer);
        // 读取失败
        if (-1 == ret)
        {
            pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

            return false;
        }
        // 读取成功
        else
        {
            // 计算累计传输数据长度
            transferred_data_len += (ret - spi_transfer[0].len);

            // 计算未发送数据长度
            remain_data_len -= current_data_len;
        }
    }

    // 数据未全部读取
    if (transferred_data_len != read_data_len)
    {
        pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

        return false;
    }

    pthread_mutex_unlock(&spi_dev_info.spi_dev_mutex);

    return true;
}
