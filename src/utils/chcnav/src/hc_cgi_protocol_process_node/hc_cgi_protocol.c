#include "hc_cgi_protocol.h"

static unsigned int return_crc32_value(unsigned int i)
{
    int j;
    unsigned int ulCRC = i;

    for (j = 8; j > 0; j--)
    {
        if (ulCRC & 1)
            ulCRC = (ulCRC >> 1) ^ 0xEDB88320;
        else
            ulCRC >>= 1;
    }

    return ulCRC;
}

/**
 * @brief 获取CGI协议的CRC32校验结果
 *
 * @param ulCount 数据长度
 * @param ucBuffer 存储协议内容
 * @param ulCRC 通常为0
 * @return uint32_t 返回crc32结果
 * */
static unsigned int get_crc32_value(unsigned int ulCount, const unsigned char *ucBuffer, unsigned int ulCRC)
{
    if (NULL == ucBuffer)
        return 0;

    unsigned int ulTemp1, ulTemp2;
    while (ulCount-- != 0)
    {
        ulTemp1 = (ulCRC >> 8) & 0x00FFFFFF;
        ulTemp2 = return_crc32_value(((int)ulCRC ^ *ucBuffer++) & 0xff);
        ulCRC = ulTemp1 ^ ulTemp2;
    }

    return (ulCRC);
}

/**
 * @brief 校验crc结果
 *
 * @param ucBuffer 存储协议内容
 * @param len 协议总长度（header+crc32+内容）
 * @return int @c 0 校验成功， @c -1 校验失败
 * */
int hc__cgi_check_crc32(const unsigned char *ucBuffer, unsigned int len)
{
    unsigned crc_result, crc_origin;
    if (ucBuffer == NULL || len < 4U)
        return -1;

    /* CGI wire format is little-endian. Decode bytewise to avoid unaligned
       access and strict-aliasing undefined behaviour. */
    crc_origin = ((unsigned int)ucBuffer[len - 4U]) |
                 ((unsigned int)ucBuffer[len - 3U] << 8U) |
                 ((unsigned int)ucBuffer[len - 2U] << 16U) |
                 ((unsigned int)ucBuffer[len - 1U] << 24U);
    crc_result = get_crc32_value(len - 4U, ucBuffer, 0U);

    if (crc_origin == crc_result)
        return 0;
    else
        return -1;
}
