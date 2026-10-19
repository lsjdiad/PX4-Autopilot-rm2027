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
 * @file nrf24l01_reg.h
 * @brief nRF24L01(+) 寄存器地址与 SPI 指令码
 *
 * 来源：mas_embedded_threadx 仓库 modules/NRF24L01/nrf24l01_reg.h
 *       （那份驱动移植自江协科技的标准库驱动，并在 NRF24L01+ 上补了动态包长 DPL 相关寄存器）。
 *       本文件只是把常量抄过来，保证飞控端与 F103 端的寄存器口径一致。
 *
 * 名词速查（给第一次接触这颗芯片的人）：
 *   - 这颗芯片用 SPI 读写"寄存器"和"FIFO"。SPI 一次传输的第一字节是"命令"，
 *     后面的字节是数据；所以下面这些宏分两类：SPI 指令码(0x6x/0xA0/0xE1...) 和 寄存器地址(0x00~0x1D)。
 *   - FIFO = 芯片内部的收/发先入先出小仓库（各 3 包深）。
 *   - Enhanced ShockBurst(ESB) = 硬件自动做的事：收完自动回 ACK、发方没收到 ACK 就自动重传。
 *     打开它就不用软件写重传逻辑，链路也更稳。
 */

#pragma once

#include <stdint.h>

/* ================= SPI 指令码 ================= */
#define NRF24L01_R_REGISTER          0x00 /* 读寄存器: 000A AAAA, 后跟 1~5 字节数据 */
#define NRF24L01_W_REGISTER          0x20 /* 写寄存器: 001A AAAA, 后跟 1~5 字节数据(仅待机/掉电模式可写) */
#define NRF24L01_R_RX_PAYLOAD        0x61 /* 读 RX 有效载荷: 后跟 1~32 字节 */
#define NRF24L01_W_TX_PAYLOAD        0xA0 /* 写 TX 有效载荷: 后跟 1~32 字节 */
#define NRF24L01_FLUSH_TX            0xE1 /* 清空 TX FIFO */
#define NRF24L01_FLUSH_RX            0xE2 /* 清空 RX FIFO */
#define NRF24L01_REUSE_TX_PL         0xE3 /* 重发最后一次载荷 */
#define NRF24L01_R_RX_PL_WID         0x60 /* 读 RX FIFO 最前面那包的宽度(DPL 模式下才有效) */
#define NRF24L01_W_ACK_PAYLOAD       0xA8 /* 写应答附带载荷(高 5 位指令 + 低 3 位通道号) */
#define NRF24L01_W_TX_PAYLOAD_NOACK  0xB0 /* 写 TX 载荷但不要求应答 */
#define NRF24L01_NOP                 0xFF /* 空操作: 只用来"顺便"读回 STATUS */
#define NRF24L01_ACTIVATE            0x50 /* 激活 FEATURE 寄存器(nRF24L01+ 必需), 后跟 0x73 */
#define NRF24L01_ACTIVATE_DATA       0x73 /* ACTIVATE 命令固定的跟随字节 */

/* ================= 寄存器地址 ================= */
#define NRF24L01_CONFIG              0x00 /* 配置寄存器(上电/收发方向/CRC/中断屏蔽) */
#define NRF24L01_EN_AA               0x01 /* 各通道"自动应答"使能 */
#define NRF24L01_EN_RXADDR           0x02 /* 各接收通道使能 */
#define NRF24L01_SETUP_AW            0x03 /* 地址宽度(3/4/5 字节) */
#define NRF24L01_SETUP_RETR          0x04 /* 自动重传: 高 4 位间隔, 低 4 位次数 */
#define NRF24L01_RF_CH               0x05 /* 射频通道(2400 + 该值 MHz) */
#define NRF24L01_RF_SETUP            0x06 /* 空中速率 / 发射功率 */
#define NRF24L01_STATUS              0x07 /* 状态(读: 各种中断标志; 写 1 清除对应位) */
#define NRF24L01_OBSERVE_TX          0x08 /* 发送统计: 重传次数 / 丢包计数 */
#define NRF24L01_RPD                 0x09 /* 接收功率检测(有个载波就置 1) */
#define NRF24L01_RX_ADDR_P0          0x0A /* 接收通道 0 地址(5 字节) */
#define NRF24L01_RX_ADDR_P1          0x0B /* 接收通道 1 地址(5 字节) */
#define NRF24L01_RX_ADDR_P2          0x0C /* 接收通道 2 地址(1 字节, 高 4 字节同 P1) */
#define NRF24L01_RX_ADDR_P3          0x0D /* 接收通道 3 地址(1 字节) */
#define NRF24L01_RX_ADDR_P4          0x0E /* 接收通道 4 地址(1 字节) */
#define NRF24L01_RX_ADDR_P5          0x0F /* 接收通道 5 地址(1 字节) */
#define NRF24L01_TX_ADDR             0x10 /* 发送地址(5 字节) */
#define NRF24L01_RX_PW_P0            0x11 /* 通道 0 定长载荷宽度(1~32) */
#define NRF24L01_RX_PW_P1            0x12 /* 通道 1 定长载荷宽度 */
#define NRF24L01_RX_PW_P2            0x13 /* 通道 2 定长载荷宽度 */
#define NRF24L01_RX_PW_P3            0x14 /* 通道 3 定长载荷宽度 */
#define NRF24L01_RX_PW_P4            0x15 /* 通道 4 定长载荷宽度 */
#define NRF24L01_RX_PW_P5            0x16 /* 通道 5 定长载荷宽度 */
#define NRF24L01_FIFO_STATUS         0x17 /* 收发 FIFO 的空/满 */
#define NRF24L01_DYNPD               0x1C /* 各通道"动态包长"使能 */
#define NRF24L01_FEATURE             0x1D /* 高级功能使能(DPL / ACK 载荷 / 无 ACK 发送) */

/* ================= CONFIG 寄存器位定义 ================= */
#define NRF24L01_CONFIG_MASK_RX_DR   (1 << 6) /* 屏蔽 RX_DR 中断 */
#define NRF24L01_CONFIG_MASK_TX_DS   (1 << 5) /* 屏蔽 TX_DS 中断 */
#define NRF24L01_CONFIG_MASK_MAX_RT  (1 << 4) /* 屏蔽 MAX_RT 中断 */
#define NRF24L01_CONFIG_EN_CRC       (1 << 3) /* 使能 CRC */
#define NRF24L01_CONFIG_CRCO         (1 << 2) /* CRC 宽度: 0=1 字节, 1=2 字节 */
#define NRF24L01_CONFIG_PWR_UP       (1 << 1) /* 上电: 0=掉电, 1=上电 */
#define NRF24L01_CONFIG_PRIM_RX      (1 << 0) /* 收发方向: 0=TX, 1=RX */

/* ================= STATUS 寄存器位定义 ================= */
#define NRF24L01_STATUS_RX_DR        (1 << 6) /* 收到一包数据 */
#define NRF24L01_STATUS_TX_DS        (1 << 5) /* 发送完成(且收到对端 ACK) */
#define NRF24L01_STATUS_MAX_RT       (1 << 4) /* 重传次数用尽(对端没应答) */
#define NRF24L01_STATUS_RX_P_NO_MASK (0x0E)   /* 数据来自哪个通道(bit3:1) */
#define NRF24L01_STATUS_TX_FULL      (1 << 0) /* TX FIFO 满 */

/* ================= FIFO_STATUS 位定义 ================= */
#define NRF24L01_FIFO_TX_REUSE       (1 << 6)
#define NRF24L01_FIFO_TX_FULL        (1 << 5) /* 1 = TX FIFO 满 */
#define NRF24L01_FIFO_TX_EMPTY       (1 << 4) /* 1 = TX FIFO 空 */
#define NRF24L01_FIFO_RX_FULL        (1 << 1) /* 1 = RX FIFO 满 */
#define NRF24L01_FIFO_RX_EMPTY       (1 << 0) /* 1 = RX FIFO 空 */

/* ================= FEATURE 寄存器位定义 ================= */
#define NRF24L01_FEATURE_EN_DPL      (1 << 2) /* 使能动态包长 */
#define NRF24L01_FEATURE_EN_ACK_PAY  (1 << 1) /* 使能"应答附带载荷" */
#define NRF24L01_FEATURE_EN_DYN_ACK  (1 << 0) /* 使能 W_TX_PAYLOAD_NOACK 指令 */

/* ================= 常用配置值 ================= */
#define NRF24L01_ADDR_WIDTH_5        0x03 /* 地址宽度 = 5 字节 */
#define NRF24L01_PAYLOAD_MAX         32   /* 单包最大载荷字节数(硬件限制) */
