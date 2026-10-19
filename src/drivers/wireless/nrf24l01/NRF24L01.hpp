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
 * @file NRF24L01.hpp
 * @brief nRF24L01 2.4GHz 无线模块驱动（飞控侧 = 常驻接收，也可以随时主动发一包）
 *
 * ============================ 这个驱动干什么 ============================
 * 一句话：把 nRF24L01 挂在硬件 SPI2 上，**开机就进接收模式**，
 * 收到数据就打印/记录；需要发的时候用 `nrf24l01 send ...` 发一包，发完自动回到接收模式。
 *
 * 对端（F103 测试板）那边用 mas_embedded 的 NRF24L01 模块，一端发一端收，
 * 两端的射频参数和载荷格式必须逐项一致，见下面 nrf24l01_config 和 nrf24l01_test_payload_s。
 *
 * ============================ 名词速查 ============================
 *  - ESB（Enhanced ShockBurst）：芯片硬件自动做的事 —— 收方收到包自动回一个 ACK，
 *    发方在设定时间内没收到 ACK 就自动重发（最多 N 次）。所以"发出去"不等于"发到了"，
 *    只有 STATUS 里的 TX_DS 置位（= 收到 ACK）才算真发到。
 *  - DPL（动态包长）：每包的字节数写在该包头部，收发自动适配；不开的话必须两端都写死同样的长度。
 *  - CE / CSN / IRQ 三根控制线：CE = "我要收发"的开关，CSN = SPI 片选，IRQ = 芯片有事件时拉低通知 MCU。
 *
 * ============================ 用法（MAVLink Console / pxh）============================
 *   nrf24l01 start            上电初始化 + 进接收模式（只做一次）
 *   nrf24l01 status           看收发计数、最近收到的一包、性能计数
 *   nrf24l01 send test        发一包标准测试载荷（16 字节，seq 自增）
 *   nrf24l01 send 01 02 03    发任意 ≤32 字节（十六进制，空格分隔）
 *   nrf24l01 regs             把关键寄存器读出来（排查"SPI 通不通、参数对不对"用）
 *   nrf24l01 stop             停止（清中断、退出工作队列）
 */

#pragma once

#include <stdint.h>

#include <drivers/drv_hrt.h>       /* hrt_abstime / hrt_absolute_time() */
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include "nrf24l01_reg.h"

/* ============================ 射频/协议参数（两端必须一致）============================
 *
 * 这些值直接照抄 mas_embedded_threadx 里 modules/NRF24L01 的默认值 —— 对端（F103）不改，
 * 飞控端也就别改。**任何一项两端不一致 = 完全收不到**（表现是发送端 TX_DS 永远不置位、
 * 接收端一直 0 包，很容易误判成"硬件坏了"）。
 */
namespace nrf24l01_config
{
static constexpr uint8_t kRfChannel{2};                     /**< 射频通道: 2400 + 2 = 2402 MHz */
static constexpr uint8_t kRfDataRate{2};                    /**< 空中速率: 1=1Mbps, 2=2Mbps */
static constexpr uint8_t kRfPower{0};                       /**< 发射功率: 0=0dBm */
static constexpr uint8_t kRetrCount{3};                     /**< 自动重传次数(0~15) */
static constexpr uint8_t kRetrDelay{0};                     /**< 重传间隔: 0=250us, 1=500us ... */
static constexpr uint8_t kAddress[5] {0x11, 0x22, 0x33, 0x44, 0x55}; /**< 5 字节地址 */

static constexpr uint32_t kSpiFrequency{2 * 1000 * 1000};   /**< SCK 先压到 ~1.5MHz：台架上是杜邦线，
                                                             *   长线在 6MHz 时 MISO 波形会烂掉（实测寄存器回读全是 0xFF）。
                                                             *   跑通后可以往上调，芯片上限 10MHz。*/
static constexpr uint16_t kTxWaitUs{3000};                  /**< 等"TX_DS/MAX_RT"的上限(us) */
static constexpr uint32_t kPollIntervalUs{50 * 1000};        /**< 兜底轮询周期(us) */
static constexpr uint32_t kPowerOnDelayUs{100 * 1000};       /**< 上电到寄存器稳定(us) */
static constexpr uint32_t kPrintIntervalUs{1000 * 1000};     /**< 收到数据打印的最小间隔(us) */
}

/* ============================ 测试载荷（与 F103 端逐字节一致）============================
 * 16 字节，小端（STM32 与 PC 都是小端，直接 memcpy 即可）：
 *   seq(4) | v1(4) | v2(4) | v3(4)
 * 对端 F103 的 nrf24l01_test_payload_t 必须是一模一样的布局。
 * packed 是为了防止编译器插填充字节（4 个 4 字节成员天然不填充，写上是为了以后加字段不出坑）。
 */
struct nrf24l01_test_payload_s {
	uint32_t seq;
	float    v1;
	float    v2;
	float    v3;
} __attribute__((packed));

/* ============================ 给 SWD 直接读的状态块 ============================
 * 本板没 USB、串口又常常被遥测占着，所以照 rc_throttle 的做法留一份"固定布局"的状态在 RAM：
 * 用 OpenOCD 不停机读这段内存就能判断链路通不通。字段全是定长整数/浮点，无指针。
 *   arm-none-eabi-nm <elf> | grep rm2027_nrf_status       # 取地址
 *   openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
 *     -c "init; mdw 0x2400xxxx 32; shutdown"
 * 改字段顺序/类型 = 改协议，记得同时升 version。
 */
struct Nrf24l01StatusBlock {
	uint32_t magic;          /**< 'NRF2' = 0x3246524E，用来确认读到的是这块内存 */
	uint32_t version;        /**< 结构体版本 */
	uint32_t run_count;      /**< Run() 被调用次数（证明驱动在跑） */
	uint32_t irq_count;      /**< IRQ 中断次数（>0 说明模块有事件、EXTI 通） */
	uint32_t rx_count;       /**< 成功解出的包数（长度匹配测试载荷才计数） */
	uint32_t rx_other;       /**< 收到但长度不匹配的包数 */
	uint32_t tx_ok;          /**< 发送并收到 ACK 的次数 */
	uint32_t tx_fail;        /**< 发送失败(没 ACK/超时)次数 */
	uint32_t last_tx_status; /**< 最近一次发送后的 STATUS 原始值 */
	uint32_t last_len;       /**< 最近一包的长度(字节) */
	uint64_t last_rx_time;   /**< 最近一次收到包的时刻(us，0=从未) */
	uint32_t last_seq;       /**< 最近一包的 seq 字段 */
	float    last_v1;        /**< 最近一包的 v1 */
	float    last_v2;        /**< 最近一包的 v2 */
	float    last_v3;        /**< 最近一包的 v3 */
	uint32_t error;          /**< 初始化/运行错误码（0=正常） */
};

extern "C" volatile Nrf24l01StatusBlock rm2027_nrf_status;

/* ============================ 驱动本体 ============================ */
class NRF24L01 : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	/** ModuleBase 的"身份证"：start/stop/status 三个子命令靠它找到我们的三个函数 */
	static Descriptor desc;

	NRF24L01();
	~NRF24L01() override;

	/** ModuleBase 接口 */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	/** 初始化：SPI 参数 + GPIO + 寄存器 + 进接收模式 + 挂 IRQ。@return true=成功 */
	bool init();

	int print_status() override;
	void request_stop() override;

	/** 从外壳命令调用：发一包（阻塞到发完或超时）。@return PX4_OK=收到 ACK */
	int send_payload(const uint8_t *data, uint8_t len);

	/** 打印关键寄存器（排查用） */
	int dump_registers();

private:
	void Run() override;

	/* ---- 底层：SPI 一帧 ---- */
	int transfer(uint16_t len);

	/* ---- 寄存器层 ---- */
	uint8_t read_reg(uint8_t reg);
	void    write_reg(uint8_t reg, uint8_t value);
	int     write_reg_checked(uint8_t reg, uint8_t value);   /**< 写+回读校验，最多重试 3 次 */
	void    write_regs(uint8_t reg, const uint8_t *buf, uint8_t len);
	uint8_t read_status();                                    /**< NOP 指令顺便读回 STATUS */
	void    activate_feature();
	void    flush_tx();
	void    flush_rx();

	/* ---- 模式与收发 ---- */
	int  configure();       /**< 按上面参数写一遍寄存器；@return PX4_OK=全部写成功 */
	void enter_rx_mode();   /**< CE 低 → PRIM_RX=1 → CE 高（常驻接收） */
	void enter_tx_mode();   /**< CE 低 → PRIM_RX=0 */
	void do_tx();           /**< 真正把 _out_tx 里的内容发出去（工作队列里执行） */
	void drain_rx();        /**< 取空 RX FIFO 并解码 */
	void handle_packet(const uint8_t *data, uint8_t len);
	void set_ce(bool high);
	void set_csn(bool low);

	static int irq_trampoline(int irq, void *context, void *arg);

	/* ---- 状态 ---- */
	struct spi_dev_s *_spi{nullptr};

	/* SPI 帧缓冲：[0]=命令，[1..]=数据。
	 * 注意：**只允许工作队列线程访问**（外壳线程要用的是 _out_tx，见下）*/
	uint8_t _tx[1 + NRF24L01_PAYLOAD_MAX] {};
	uint8_t _rx[1 + NRF24L01_PAYLOAD_MAX] {};

	/* 外壳命令要发的载荷（和工作队列的帧缓冲分开，避免两个线程同时改一块内存）*/
	uint8_t _out_tx[NRF24L01_PAYLOAD_MAX] {};
	uint8_t _out_len{0};
	volatile bool _tx_request{false};    /**< 外壳线程置位 → 工作队列来发 */
	volatile int  _tx_result{PX4_ERROR}; /**< 发送结果（PX4_OK=收到 ACK）*/

	/* 统计 */
	uint32_t _run_count{0};
	uint32_t _irq_count{0};
	uint32_t _rx_count{0};
	uint32_t _rx_other{0};
	uint32_t _tx_ok{0};
	uint32_t _tx_fail{0};
	uint32_t _last_tx_status{0};
	uint8_t  _last_len{0};
	uint32_t _test_seq{0};
	nrf24l01_test_payload_s _last_payload {};
	hrt_abstime _last_rx_time{0};
	hrt_abstime _last_print_time{0};

	/* 性能计数（`nrf24l01 status` 里会打印；pxh 的 `status` 也能看到）*/
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED,  MODULE_NAME ": cycle")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};
};
