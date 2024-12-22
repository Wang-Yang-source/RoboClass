#ifndef TRANSLATE_H
#define TRANSLATE_H

#include <Arduino.h>

/**
 * @brief 处理传入的串行数据以控制LED并打印命令。
 *
 * 该函数从串行输入读取数据，直到遇到换行符。
 * 然后将接收到的数据转换为表示角度的浮点数。
 * 根据角度值，它执行以下操作：
 * - 如果角度大于30，则打开LED并打印右转命令。
 * - 如果角度小于-30，则打开LED并打印左转命令。
 * - 否则，关闭LED并打印运行命令。
 *
 * @note 确保在调用此函数之前正确初始化串行通信。
 * @note `ledPin` 应该被定义并配置为输出引脚。
 */

float translate() {
  float angle = 0.0;
  if (Serial.available() > 0) {
    String receivedData = Serial.readStringUntil('\n'); // 读取直到换行符的传入数据
    angle = receivedData.toFloat(); // 将接收到的数据转换为浮点数
  }
  return angle;
}

/**
 * @brief 初始化串行通信并配置LED引脚。
 *
 * 该函数以指定的波特率设置串行通信
 * 并将LED引脚配置为输出。
 *
 * @param baudRate 串行通信的波特率。
 */

#endif // TRANSLATE_H
