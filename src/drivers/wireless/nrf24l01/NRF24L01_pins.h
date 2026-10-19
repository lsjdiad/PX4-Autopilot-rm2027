/****************************************************************************
 *
 *   Copyright (c) 2026 RM2027 Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file NRF24L01_pins.h
 * @brief nRF24L01 在本板（rm2027/fmu-h743）上占用的引脚 —— **临时引脚，硬件改版后只改这个文件**
 *
 * ============================ 一句话 ============================
 * 本板现在还没有 nRF24L01 的专用插座，所以先借用**两个空闲的 2.54mm 排针座**：
 *   H1 座（3 信号 + 1 GND）= 硬件 SPI2 的三根线
 *   H2 座（3 信号 + 1 GND）= CSN / CE / IRQ 三根普通 GPIO
 * 依据：原理图网表 FMU_netlist.net（见 docs/vl53l0x/FMU_netlist.net）
 *   SPI2_SCK  = U1-84 PD3 ↔ H1-1        SPI2_MOSI = U1-18 PC3 ↔ H1-2
 *   SPI2_MISO = U1-17 PC2 ↔ H1-3        H1-4 = GND
 *   SPI3_SCK  = U1-78 PC10 ↔ H2-1       SPI3_MISO = U1-79 PC11 ↔ H2-2
 *   SPI3_MOSI = U1-36 PB2  ↔ H2-3       H2-4 = GND
 *
 * ============================ 飞控侧接线表 ============================
 *   nRF24L01 丝印    接到           本板引脚     备注
 *   ------------------------------------------------------------------
 *   GND         →   H1-4 / H2-4     GND          必须共地
 *   VCC         →   见下方"供电"                   3.3V（带 LDO 的模块才可 5V）
 *   SCK         →   H1-1            PD3          SPI2_SCK（AF，硬件 SPI）
 *   MOSI        →   H1-2            PC3          SPI2_MOSI
 *   MISO        →   H1-3            PC2          SPI2_MISO
 *   CSN         →   H2-1            PC10         片选，低有效（普通 GPIO）
 *   CE          →   H2-2            PC11         收发使能（普通 GPIO）
 *   IRQ         →   H2-3            PB2          模块中断输出（下降沿）
 *
 * 供电：本板所有插座上**只有 5V，没有 3.3V**（与之前 VL53L0X 那次同一结论）。
 *   - 模块若是常见"带 LDO 的转接板"（板上有一颗 662K / XC6206 之类的小稳压），
 *     则 VCC 可以接 5V（从 U16~U21 任意一个座子的第 1 脚取，H1/H2 上没有 5V）。
 *   - 模块若是**光板 8 脚模块（无 LDO）**，必须给 3.3V：或从 F103 测试板的 3.3V 取电（共地），
 *     或另接一路 3.3V；**绝对不要接 5V**，会烧芯片。
 *   - 无论哪种，模块 VCC-GND 之间并 100nF + 10µF 电容（发射瞬间电流大，不平滑会丢包）。
 *
 * ============================ 为什么这样选 ============================
 * 1. nRF24L01 的 SPI 最高只支持 10MHz，且**必须模式 0**（CPOL=0/CPHA=0）→ 用硬件 SPI2，
 *    频率在驱动里设成 8MHz（NuttX 会自动挑不超过它的分频档），比软件模拟 SPI 稳得多。
 * 2. SPI2 的 SCK/MISO/MOSI 是**芯片 AF 定死**的（PD3/PC2/PC3，见 nuttx-config/include/board.h
 *    的 GPIO_SPI2_*），不能随便换脚；换这三根等于换总线。
 * 3. CSN/CE/IRQ 是普通 GPIO，H2 座空着就先用它；PC10/PC11/PB2 在本板网表里都是"单点网络"
 *    （只连到排针，没接别的器件）→ 不会和别的功能打架。
 * 4. IRQ 需要能中断：PB2 = EXTI2，本板 EXTI2 线目前没人用（PD2 是 UART5_RX 的串口输入，不占 EXTI）。
 *
 * ============================ 以后硬件改版要改哪里 ============================
 * ① 只换 CSN/CE/IRQ 三根脚 → 改下面三行 GPIO 定义即可（任意空闲 GPIO 都行，IRQ 要有 EXTI）。
 * ② 换成另一条 SPI 总线 → 改 NRF24L01_SPI_BUS，并在 board 的 nuttx-config/nsh/defconfig 里
 *    使能对应的 CONFIG_STM32H7_SPIx=y，同时在 nuttx-config/include/board.h 里给那条总线
 *    选好 SCK/MISO/MOSI 引脚（本板 SPI2/SPI3/SPI4 都有现成的 GPIO_SPIx_* 定义）。
 * ③ 若改版后 SPI2 的引脚也变了（例如换成 SPI4），**不要**只改这里：先把 board.h 里
 *    该总线的 GPIO_SPIx_SCK/MISO/MOSI 指到新脚，再让硬件按新脚布线。
 */

#pragma once

#include <board_config.h>   /* PX4_MAKE_GPIO_* / GPIO_PORTC / GPIO_PIN10 等 */

/* 用哪条硬件 SPI 总线（1 起数）：
 * 2 = SPI2 = H1 座（SCK=PD3 / MISO=PC2 / MOSI=PC3），由 board.h 的 GPIO_SPI2_* 决定 */
#define NRF24L01_SPI_BUS     2

/* CSN（片选，低有效，空闲时必须高）：H2-1 = PC10 */
#define NRF24L01_CSN_GPIO    PX4_MAKE_GPIO_OUTPUT_SET(GPIO_PORTC | GPIO_PIN10)

/* CE（收发使能，空闲时必须低）：H2-2 = PC11 */
#define NRF24L01_CE_GPIO     PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_PORTC | GPIO_PIN11)

/* IRQ（模块有事件时拉低，下降沿中断）：H2-3 = PB2（EXTI2），带内部上拉 */
#define NRF24L01_IRQ_GPIO    PX4_MAKE_GPIO_INPUT(GPIO_PORTB | GPIO_PIN2)
