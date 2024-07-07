/**
 * @file      : spi.h
 * @brief     : Linux平台SPI驱动头文件
 * @author    : huenrong (huenrong1028@outlook.com)
 * @date      : 2023-01-18 15:10:17
 *
 * @copyright : Copyright (c) 2023 huenrong
 *
 * @history   : date       author          description
 *              2023-01-18 huenrong        创建文件
 *
 */

#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

// SPI单次最大传输长度
#define SPI_MAX_TRANSFER_LEN 4096

#define SPI_CPHA 0x01
#define SPI_CPOL 0x02

// SPI模式定义
// 其实在文件"linux/spi/spidev.h"中有定义
typedef enum
{
    E_SPI_MODE_0 = (0 | 0),
    E_SPI_MODE_1 = (0 | SPI_CPHA),
    E_SPI_MODE_2 = (SPI_CPOL | 0),
    E_SPI_MODE_3 = (SPI_CPOL | SPI_CPHA),
} spi_mode_e;

/**
 * @brief  打开SPI设备
 * @param  spi_dev_name: 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @param  spi_mode    : 输入参数, SPI模式
 * @param  spi_speed   : 输入参数, SPI设备默认最大速度(单位: Hz)
 * @param  spi_bits    : 输入参数, SPI设备字长
 * @return true : 成功
 * @return false: 失败
 */
bool spi_open(const char *spi_dev_name, const spi_mode_e spi_mode, const uint32_t spi_speed, const uint8_t spi_bits);

/**
 * @brief  设置SPI单次最大传输长度, 未设置或设置值为0会使用内部默认值SPI_MAX_TRANSFER_LEN
 * @param  max_transfer_len: 输入参数, 单次最大传输长度
 */
void spi_set_max_transfer_len(const uint32_t max_transfer_len);

/**
 * @brief  关闭SPI设备
 * @param  spi_dev_name: 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @return true : 成功
 * @return false: 失败
 */
bool spi_close(const char *spi_dev_name);

/**
 * @brief  向无寄存器地址的SPI从设备发送1字节数据
 * @param  spi_dev_name: 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @param  write_data  : 输入参数, 待发送数据
 * @return true : 成功
 * @return false: 失败
 */
bool spi_write_byte(const char *spi_dev_name, const uint8_t write_data);

/**
 * @brief  向无寄存器地址的SPI从设备发送n字节数据(n > 1)
 * @param  spi_dev_name  : 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @param  write_data    : 输入参数, 待发送数据
 * @param  write_data_len: 输入参数, 待发送数据长度
 * @return true : 成功
 * @return false: 失败
 */
bool spi_write_nbyte(const char *spi_dev_name, const uint8_t *write_data, const uint32_t write_data_len);

/**
 * @brief  从无寄存器地址的SPI从设备读取1字节数据
 * @param  read_data   : 输出参数, 读取到的数据
 * @param  spi_dev_name: 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @return true : 成功
 * @return false: 失败
 */
bool spi_read_byte(uint8_t *read_data, const char *spi_dev_name);

/**
 * @brief  从无寄存器地址的SPI从设备读取n字节数据(n > 1)
 * @param  read_data    : 输出参数, 读取到的数据
 * @param  spi_dev_name : 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @param  read_data_len: 输入参数, 指定读取的长度
 * @return true : 成功
 * @return false: 失败
 */
bool spi_read_nbyte(uint8_t *read_data, const char *spi_dev_name, const uint16_t read_data_len);

/**
 * @brief  向有寄存器地址的SPI从设备发送1字节数据
 * @param  spi_dev_name: 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @param  reg_addr    : 输入参数, 寄存器地址
 * @param  write_data  : 输入参数, 待发送数据
 * @return true : 成功
 * @return false: 失败
 */
bool spi_write_byte_sub(const char *spi_dev_name, const uint8_t reg_addr, const uint8_t write_data);

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
                         const uint8_t *write_data, const uint32_t write_data_len);

/**
 * @brief  从有寄存器地址的SPI从设备读取1字节数据
 * @param  read_data   : 输出参数, 读取到的数据
 * @param  spi_dev_name: 输入参数, SPI设备名(如: /dev/spidev2.0)
 * @param  reg_addr    : 输入参数, 寄存器地址
 * @return true : 成功
 * @return false: 失败
 */
bool spi_read_byte_sub(uint8_t *read_data, const char *spi_dev_name, const uint8_t reg_addr);

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
                        const uint8_t reg_addr, const uint16_t read_data_len);

#ifdef __cplusplus
}
#endif

#endif // __SPI_H
