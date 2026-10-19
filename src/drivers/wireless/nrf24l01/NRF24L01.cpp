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
 * @file NRF24L01.cpp
 * @brief nRF24L01 驱动的实现（寄存器时序照抄 mas_embedded_threadx/modules/NRF24L01，去掉 ThreadX 那层）
 *
 * 数据流（收）：模块收到包 → IRQ 拉低 → EXTI 中断 → 叫醒工作队列 → 读 FIFO → 解码 → 打印/记状态
 * 数据流（发）：nrf24l01 send ... → 把载荷交给工作队列 → 切 TX → 写 TX FIFO → CE 拉高 → 等 TX_DS → 回 RX
 *
 * 一句话理解寄存器配置：先把 CONFIG 打开 CRC/上电，再打开通道 0 的自动应答和 5 字节地址，
 * 然后 ACTIVATE 打开 FEATURE 里的"动态包长"，写地址，清 FIFO，最后进 RX 常驻收。
 */

#include "NRF24L01.hpp"

#include <cstdlib>
#include <cstring>

#include <board_config.h>          /* 板级 GPIO / PX4_MAKE_GPIO_* / px4_arch_* */
#include <nuttx/spi/spi.h>         /* SPI_EXCHANGE / SPI_SETMODE / SPIDEV_MODE0 */

#include <hardware/stm32h7x3xx_rcc.h>   /* STM32_RCC_APB1LENR / RCC_APB1LENR_SPI2EN */

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>   /* time_literals（200_ms 这种写法）*/
#include <unistd.h>

#include "NRF24L01_pins.h"
#include "nrf24l01_reg.h"

using namespace time_literals;

/* SWD 可读的状态块（符号名固定，别改） */
extern "C" { volatile Nrf24l01StatusBlock rm2027_nrf_status; }

ModuleBase::Descriptor NRF24L01::desc{task_spawn, custom_command, print_usage};

NRF24L01::NRF24L01() :
	ModuleBase(),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::SPI2)
{
	rm2027_nrf_status.magic = 0x3246524E; /* 'NRF2' */
	rm2027_nrf_status.version = 1;
}

NRF24L01::~NRF24L01()
{
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

/* ==========================================================================
 * 底层：一次 SPI 帧（把 _tx 发出去，把回读放到 _rx），CSN 由我们自己拉
 * ========================================================================== */
void NRF24L01::set_csn(bool low)
{
	px4_arch_gpiowrite(NRF24L01_CSN_GPIO, low ? 0 : 1);
}

void NRF24L01::set_ce(bool high)
{
	px4_arch_gpiowrite(NRF24L01_CE_GPIO, high ? 1 : 0);
}

int NRF24L01::transfer(uint16_t len)
{
	set_csn(true);
	SPI_EXCHANGE(_spi, _tx, _rx, len);
	set_csn(false);
	return PX4_OK;
}

/**
 * @brief 发一条命令帧：_tx[0]=命令，_tx[1..datalen]=数据（调用方先填好）
 *
 * nRF24L01 的 SPI 有个特点：**发命令字节的同时**，芯片会把它当前的 STATUS 从 MISO 吐回来，
 * 所以命令帧的 _rx[0] 就是"顺便读到的状态"（读寄存器、发数据时都用得到）。
 */
#define NRF_CMD(cmd_byte, datalen) do { _tx[0] = (cmd_byte); transfer(1 + (datalen)); } while (0)

/* ==========================================================================
 * 寄存器层（下面这些成员函数都直接操作 _tx/_rx，只允许在工作队列线程里调用）
 * ========================================================================== */

uint8_t NRF24L01::read_reg(uint8_t reg)
{
	_tx[1] = NRF24L01_NOP; /* 占位字节，只为产生时钟；内容芯片不关心 */
	NRF_CMD(NRF24L01_R_REGISTER | reg, 1);
	return _rx[1];
}

void NRF24L01::write_reg(uint8_t reg, uint8_t value)
{
	_tx[1] = value;
	NRF_CMD(NRF24L01_W_REGISTER | reg, 1);
}

int NRF24L01::write_reg_checked(uint8_t reg, uint8_t value)
{
	for (int i = 0; i < 3; i++) {
		write_reg(reg, value);

		if (read_reg(reg) == value) {
			return PX4_OK;
		}
	}

	PX4_ERR("reg 0x%02x write verify failed: want 0x%02x got 0x%02x (查 SPI 接线/供电)",
		reg, value, read_reg(reg));
	rm2027_nrf_status.error = 0x100 | reg;
	return PX4_ERROR;
}

void NRF24L01::write_regs(uint8_t reg, const uint8_t *buf, uint8_t len)
{
	memcpy(&_tx[1], buf, len);
	NRF_CMD(NRF24L01_W_REGISTER | reg, len);
}

uint8_t NRF24L01::read_status()
{
	_tx[0] = NRF24L01_NOP;
	transfer(1);
	return _rx[0];
}

void NRF24L01::activate_feature()
{
	_tx[1] = NRF24L01_ACTIVATE_DATA;
	NRF_CMD(NRF24L01_ACTIVATE, 1);
}

void NRF24L01::flush_tx()
{
	NRF_CMD(NRF24L01_FLUSH_TX, 0);
}

void NRF24L01::flush_rx()
{
	NRF_CMD(NRF24L01_FLUSH_RX, 0);
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */
bool NRF24L01::init()
{
	/* 0. ★ 先打开 SPI2 的外设时钟（APB1 上的 SPI2EN，bit14）。
	 *
	 * 为什么驱动里要自己开：本板实测（2026-10-19 上板）发现，
	 *   —— 光把 CONFIG_STM32H7_SPI2=y 编进固件**不够**：启动后读 RCC_APB1LENR = 0x540001，
	 *      bit14(SPI2EN) 是 0（同时 USART3/UART5/I2C2 等都在）。
	 *   —— SPI2 没时钟 → 它的寄存器读回永远是 0 → NuttX 的 SPI 驱动会卡在
	 *      "等 TXP 标志" 的死循环里（PC 停在 spi_send，整个飞控连 MAVLink 都停了，
	 *      而且看不出任何 hardfault —— 极易误判成"驱动写错了"）。
	 *   —— 用调试器手动把这一位置 1 之后，SPI2 立刻活了（SR 的 TXP=1），驱动正常。
	 * PX4 对定时器就是这么办的（platforms/nuttx/src/px4/stm/stm32_common/io_pins/io_timer.c:746
	 * 用 modifyreg32 打开时钟位），这里照抄同样的做法。
	 * ⚠️ 硬件改版/换 SPI 总线时，这一行要跟着换成对应总线的时钟位（SPI3 = APB1 bit15，SPI4 = APB1 bit29）。*/
	*(volatile uint32_t *)STM32_RCC_APB1LENR |= RCC_APB1LENR_SPI2EN;

	/* 0.5 备注（2026-10-19 上板实测更正）：
	 *   一开始怀疑 SPI123 内核时钟源配错（board.h 说 PLL2P=96MHz，而寄存器读回 0x11000 看着像
	 *   SPI123SEL=0），后来核对头文件发现 RCC_D2CCIP1R_SPI123SEL_SHIFT = 12（不是 4），
	 *   0x11000 的 bit12 正是 SPI123SEL=1 → **就是 PLL2P，和 board.h 一致，没有配错**。
	 *   ⇒ 运行期不需要改这个寄存器。SCK = 96MHz / 分频（请求 8MHz 时分频 16 → 实际 6MHz）。*/

	/* 1. 拿 SPI 总线句柄并强制参数：模式 0、8 位、8MHz(≤10MHz)
	 *    （stm32_spibus_initialize 会顺手把 SPI2 的 SCK/MISO/MOSI 三根脚配成 AF）*/
	_spi = px4_spibus_initialize(NRF24L01_SPI_BUS);

	if (_spi == nullptr) {
		PX4_ERR("SPI%d init failed", NRF24L01_SPI_BUS);
		rm2027_nrf_status.error = 1;
		return false;
	}

	SPI_SETMODE(_spi, SPIDEV_MODE0);
	SPI_SETBITS(_spi, 8);
	SPI_SETFREQUENCY(_spi, nrf24l01_config::kSpiFrequency);

	/* 2. CSN / CE / IRQ 三根 GPIO
	 *    CSN 空闲高、CE 空闲低（芯片只在待机模式能改寄存器，CE 高会一直收发）*/
	px4_arch_configgpio(NRF24L01_CSN_GPIO);
	px4_arch_configgpio(NRF24L01_CE_GPIO);
	px4_arch_configgpio(NRF24L01_IRQ_GPIO);
	set_csn(false);
	set_ce(false);

	/* 3. 等模块上电稳定（掉电→待机要 1.5ms，这里给足）*/
	usleep(nrf24l01_config::kPowerOnDelayUs);

	/* 4. 写寄存器 */
	if (configure() != PX4_OK) {
		return false;
	}

	/* 5. 进接收模式；IRQ 下降沿叫醒工作队列；再挂一个 50ms 兜底轮询
	 *    （IRQ 是"电平事件"不是"计数器"，假如中断队列拥堵漏掉一次边沿，
	 *     兜底轮询还能把 FIFO 里的包取出来）*/
	enter_rx_mode();

	const int ret = px4_arch_gpiosetevent(NRF24L01_IRQ_GPIO, false, true, false, &NRF24L01::irq_trampoline, this);

	if (ret != PX4_OK) {
		/* 不致命：没有中断还有兜底轮询，只是延迟大一点 */
		PX4_WARN("IRQ (EXTI) register failed: %d, 退化为 %ums 轮询接收", ret,
			 (unsigned)(nrf24l01_config::kPollIntervalUs / 1000));
	}

	ScheduleDelayed(nrf24l01_config::kPollIntervalUs);

	/* 自检：FEATURE 读回 0 说明 ACTIVATE 没成功（动态包长不可用）*/
	if (read_reg(NRF24L01_FEATURE) == 0x00) {
		PX4_ERR("FEATURE=0, ACTIVATE 失败 —— SPI 通了但高级功能没打开，查供电/接线");
		rm2027_nrf_status.error = 0x200;
	}

	PX4_INFO("NRF24L01 就绪: SPI%d %ukHz, ch=%u %uMbps addr=%02x:%02x:%02x:%02x:%02x, 常驻接收",
		 NRF24L01_SPI_BUS, (unsigned)(nrf24l01_config::kSpiFrequency / 1000),
		 (unsigned)nrf24l01_config::kRfChannel, (unsigned)nrf24l01_config::kRfDataRate,
		 nrf24l01_config::kAddress[0], nrf24l01_config::kAddress[1], nrf24l01_config::kAddress[2],
		 nrf24l01_config::kAddress[3], nrf24l01_config::kAddress[4]);

	return true;
}

int NRF24L01::configure()
{
	int ret = PX4_OK;

	/* CONFIG: 使能 CRC(2 字节) + 上电；PRIM_RX 先不管（进模式时再改）
	 * ⚠️ CRC 宽度两端必须一致，否则硬件 CRC 校验全失败、表现为"完全收不到" */
	ret |= write_reg_checked(NRF24L01_CONFIG, NRF24L01_CONFIG_EN_CRC | NRF24L01_CONFIG_CRCO |
				 NRF24L01_CONFIG_PWR_UP);

	/* ★ 写完 PWR_UP=1 必须等 ≥1.5ms（数据手册的 Power up 时间），让内部振荡器起振。
	 * 不等的话：紧接着访问的第一个寄存器会写不进/读回 0 —— 实测就是 EN_AA(0x01)
	 * 写 0x01 读回 0x00（再往后 2ms 之外的寄存器又都正常），非常像"接触不良"，
	 * 但其实是芯片正在上电。 */
	usleep(2000);

	ret |= write_reg_checked(NRF24L01_EN_AA, 0x01);                 /* 通道 0 自动应答 */
	ret |= write_reg_checked(NRF24L01_EN_RXADDR, 0x01);             /* 只开通道 0 */
	ret |= write_reg_checked(NRF24L01_SETUP_AW, NRF24L01_ADDR_WIDTH_5);

	/* 自动重传：间隔 250us*(DELAY+1)，次数 COUNT */
	const uint8_t retr = (uint8_t)((nrf24l01_config::kRetrDelay << 4) | (nrf24l01_config::kRetrCount & 0x0F));
	ret |= write_reg_checked(NRF24L01_SETUP_RETR, retr);

	ret |= write_reg_checked(NRF24L01_RF_CH, nrf24l01_config::kRfChannel);

	/* RF_SETUP: bit3=1 → 2Mbps；bit2:1 = 发射功率 */
	uint8_t rf_setup = 0;

	if (nrf24l01_config::kRfDataRate == 2) {
		rf_setup |= (1 << 3);
	}

	rf_setup |= (uint8_t)((nrf24l01_config::kRfPower & 0x03) << 1);
	ret |= write_reg_checked(NRF24L01_RF_SETUP, rf_setup);

	/* 动态包长(DPL)：必须先 ACTIVATE 才能写 FEATURE 寄存器
	 * ⚠️ 这里连 EN_ACK_PAY 一起打开（值 = 0x06），是为了和 mas_embedded 那份已验证的配置逐位一致：
	 *    只开 EN_DPL 也能通信，但两块板对读 FEATURE 时会不一样，排查时容易误判。*/
	activate_feature();
	ret |= write_reg_checked(NRF24L01_FEATURE, NRF24L01_FEATURE_EN_DPL | NRF24L01_FEATURE_EN_ACK_PAY);
	ret |= write_reg_checked(NRF24L01_DYNPD, 0x01);                 /* 通道 0 动态包长 */

	/* 地址：TX_ADDR 与 RX_ADDR_P0 用同一个（自动 ACK 要求发方的 P0 = 自己的 TX_ADDR）*/
	write_regs(NRF24L01_TX_ADDR, nrf24l01_config::kAddress, 5);
	write_regs(NRF24L01_RX_ADDR_P0, nrf24l01_config::kAddress, 5);

	/* 清 FIFO 与中断标志 */
	flush_tx();
	flush_rx();
	write_reg(NRF24L01_STATUS, NRF24L01_STATUS_RX_DR | NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);

	if (ret != PX4_OK) {
		rm2027_nrf_status.error = 0x300;
	}

	return ret;
}

void NRF24L01::enter_rx_mode()
{
	/* 进 RX 的顺序有讲究：CE 拉低 → 改 PRIM_RX → CE 拉高
	 * （CE 高的时候芯片在收发，改寄存器会被忽略）*/
	set_ce(false);

	uint8_t config = read_reg(NRF24L01_CONFIG);
	config |= (NRF24L01_CONFIG_EN_CRC | NRF24L01_CONFIG_CRCO | NRF24L01_CONFIG_PWR_UP | NRF24L01_CONFIG_PRIM_RX);
	write_reg_checked(NRF24L01_CONFIG, config);

	set_ce(true); /* 常驻接收 */
}

void NRF24L01::enter_tx_mode()
{
	set_ce(false);

	uint8_t config = read_reg(NRF24L01_CONFIG);
	config |= (NRF24L01_CONFIG_EN_CRC | NRF24L01_CONFIG_CRCO | NRF24L01_CONFIG_PWR_UP);
	config &= (uint8_t)~NRF24L01_CONFIG_PRIM_RX;
	write_reg_checked(NRF24L01_CONFIG, config);
}

/* ==========================================================================
 * 中断（IRQ 下降沿）→ 只负责叫醒工作队列
 * ========================================================================== */
int NRF24L01::irq_trampoline(int irq, void *context, void *arg)
{
	NRF24L01 *instance = static_cast<NRF24L01 *>(arg);

	if (instance != nullptr) {
		instance->_irq_count++;
		rm2027_nrf_status.irq_count = instance->_irq_count;
		instance->ScheduleNow();   /* 中断里不做 SPI：交给工作队列线程 */
	}

	return PX4_OK;
}

/* ==========================================================================
 * 工作队列主循环
 * ========================================================================== */
void NRF24L01::Run()
{
	perf_begin(_cycle_perf);

	_run_count++;
	rm2027_nrf_status.run_count = _run_count;

	/* ① 有发请求就先发（半双工：发的时候收不了，所以发完立刻回接收模式）*/
	if (_tx_request) {
		do_tx();
	}

	/* ② 把 RX FIFO 取空（一次中断可能对应多包）*/
	drain_rx();

	perf_end(_cycle_perf);
	perf_count(_interval_perf);

	/* ③ 兜底重挂（IRQ 边沿漏了也能收）*/
	if (!should_exit()) {
		ScheduleDelayed(nrf24l01_config::kPollIntervalUs);
	}
}

void NRF24L01::do_tx()
{
	enter_tx_mode();
	flush_tx();

	/* 组帧：命令 + 载荷 */
	_tx[0] = NRF24L01_W_TX_PAYLOAD;
	memcpy(&_tx[1], _out_tx, _out_len);
	transfer(1 + _out_len);

	/* CE 拉高触发发送；ESB 会自己发 + 等 ACK + 必要时重传，
	 * 结束后 STATUS 里 TX_DS(成功) 或 MAX_RT(重传用尽) 会置位 */
	set_ce(true);

	const hrt_abstime start = hrt_absolute_time();
	uint8_t status = 0;

	while (hrt_elapsed_time(&start) < nrf24l01_config::kTxWaitUs) {
		status = read_status();

		if (status & (NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT)) {
			break;
		}
	}

	set_ce(false);

	/* 收尾：清标志（写 1 清），失败时先把 TX FIFO 里剩下的包扔掉 */
	const bool ok = (status & NRF24L01_STATUS_TX_DS) != 0;

	if (ok) {
		_tx_ok++;
		rm2027_nrf_status.tx_ok = _tx_ok;

	} else {
		_tx_fail++;
		rm2027_nrf_status.tx_fail = _tx_fail;
		flush_tx();
	}

	write_reg(NRF24L01_STATUS, NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);

	_last_tx_status = status;
	rm2027_nrf_status.last_tx_status = status;

	_tx_result = ok ? PX4_OK : PX4_ERROR;
	_tx_request = false;

	/* 回到常驻接收 */
	enter_rx_mode();
}

void NRF24L01::drain_rx()
{
	/* RX FIFO 最多缓存 3 包，按 3 次取空；取完为止 */
	for (int i = 0; i < 3; i++) {
		const uint8_t status = read_status();

		if (!(status & NRF24L01_STATUS_RX_DR)) {
			break;
		}

		/* 动态包长：先问这包多长，再决定读几个字节 */
		_tx[1] = NRF24L01_NOP;
		NRF_CMD(NRF24L01_R_RX_PL_WID, 1);
		const uint8_t len = _rx[1];

		if (len > 0 && len <= NRF24L01_PAYLOAD_MAX) {
			memset(&_tx[1], NRF24L01_NOP, len); /* 占位，只为时钟 */
			NRF_CMD(NRF24L01_R_RX_PAYLOAD, len);
			handle_packet(&_rx[1], len);
		}

		/* 无论长度合不合法都要清标志 + 清 FIFO，否则会卡住不再产生新中断 */
		write_reg(NRF24L01_STATUS, NRF24L01_STATUS_RX_DR);
		flush_rx();
	}
}

void NRF24L01::handle_packet(const uint8_t *data, uint8_t len)
{
	_last_len = len;
	rm2027_nrf_status.last_len = len;

	if (len != sizeof(nrf24l01_test_payload_s)) {
		/* 长度不匹配：不是我们能解的格式（对端注册的数据项个数/类型对不上就会这样）*/
		_rx_other++;
		rm2027_nrf_status.rx_other = _rx_other;
		return;
	}

	nrf24l01_test_payload_s payload {};
	memcpy(&payload, data, sizeof(payload));

	_last_payload = payload;
	_last_rx_time = hrt_absolute_time();
	_rx_count++;

	rm2027_nrf_status.rx_count = _rx_count;
	rm2027_nrf_status.last_rx_time = _last_rx_time;
	rm2027_nrf_status.last_seq = payload.seq;
	rm2027_nrf_status.last_v1 = payload.v1;
	rm2027_nrf_status.last_v2 = payload.v2;
	rm2027_nrf_status.last_v3 = payload.v3;

	/* 打印限速：最多 1 秒一条，别把控制台刷爆 */
	if (hrt_elapsed_time(&_last_print_time) >= nrf24l01_config::kPrintIntervalUs) {
		_last_print_time = hrt_absolute_time();
		PX4_INFO("nrf24l01 rx: seq=%lu v1=%.3f v2=%.3f v3=%.3f (%u 包)",
			 (unsigned long)payload.seq, (double)payload.v1, (double)payload.v2, (double)payload.v3,
			 (unsigned)_rx_count);
	}
}

/* ==========================================================================
 * 外壳接口
 * ========================================================================== */
int NRF24L01::send_payload(const uint8_t *data, uint8_t len)
{
	if (len == 0 || len > NRF24L01_PAYLOAD_MAX) {
		PX4_ERR("长度必须在 1~%u 字节之间", (unsigned)NRF24L01_PAYLOAD_MAX);
		return PX4_ERROR;
	}

	if (_tx_request) {
		PX4_ERR("上一包还没发完");
		return PX4_ERROR;
	}

	memcpy(_out_tx, data, len);
	_out_len = len;
	_tx_result = PX4_ERROR;
	_tx_request = true;

	ScheduleNow(); /* 让工作队列去发 */

	/* 等工作队列把 _tx_request 清掉（正常几毫秒内；最多等 200ms）*/
	const hrt_abstime start = hrt_absolute_time();

	while (_tx_request && hrt_elapsed_time(&start) < 200_ms) {
		usleep(1000);
	}

	if (_tx_request) {
		_tx_request = false;
		PX4_ERR("发送超时（工作队列没跑？）");
		return PX4_ERROR;
	}

	return _tx_result;
}

int NRF24L01::dump_registers()
{
	PX4_INFO("CONFIG=0x%02x EN_AA=0x%02x EN_RXADDR=0x%02x SETUP_AW=0x%02x",
		 read_reg(NRF24L01_CONFIG), read_reg(NRF24L01_EN_AA), read_reg(NRF24L01_EN_RXADDR),
		 read_reg(NRF24L01_SETUP_AW));
	PX4_INFO("SETUP_RETR=0x%02x RF_CH=0x%02x RF_SETUP=0x%02x FEATURE=0x%02x DYNPD=0x%02x",
		 read_reg(NRF24L01_SETUP_RETR), read_reg(NRF24L01_RF_CH), read_reg(NRF24L01_RF_SETUP),
		 read_reg(NRF24L01_FEATURE), read_reg(NRF24L01_DYNPD));
	PX4_INFO("STATUS=0x%02x FIFO_STATUS=0x%02x OBSERVE_TX=0x%02x(高4位=重传次数 低4位=丢包计数)",
		 read_status(), read_reg(NRF24L01_FIFO_STATUS), read_reg(NRF24L01_OBSERVE_TX));

	/* 地址回读：5 字节，用来确认地址真的写进去了（读不回来 = SPI 不通）*/
	memset(&_tx[1], NRF24L01_NOP, 5);
	NRF_CMD(NRF24L01_R_REGISTER | NRF24L01_RX_ADDR_P0, 5);
	PX4_INFO("RX_ADDR_P0=%02x:%02x:%02x:%02x:%02x", _rx[1], _rx[2], _rx[3], _rx[4], _rx[5]);

	return PX4_OK;
}

int NRF24L01::print_status()
{
	PX4_INFO("状态: run=%lu irq=%lu rx=%lu rx_other=%lu tx_ok=%lu tx_fail=%lu error=0x%lx",
		 (unsigned long)_run_count, (unsigned long)_irq_count, (unsigned long)_rx_count,
		 (unsigned long)_rx_other, (unsigned long)_tx_ok, (unsigned long)_tx_fail,
		 (unsigned long)rm2027_nrf_status.error);

	if (_last_rx_time != 0) {
		PX4_INFO("最近一包: seq=%lu v1=%.3f v2=%.3f v3=%.3f 长度=%u 在 %.2fs 前",
			 (unsigned long)_last_payload.seq, (double)_last_payload.v1, (double)_last_payload.v2,
			 (double)_last_payload.v3, (unsigned)_last_len, (double)hrt_elapsed_time(&_last_rx_time) / 1e6);
		PX4_INFO("最近一次发送 STATUS=0x%02x（bit5=TX_DS 收到ACK, bit4=MAX_RT 重传用尽）",
			 (unsigned)_last_tx_status);

	} else {
		PX4_INFO("还没收到过任何包（irq=%lu 若为 0 → 检查 IRQ 接线/对端有没有在发）",
			 (unsigned long)_irq_count);
	}

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	return PX4_OK;
}

void NRF24L01::request_stop()
{
	ScheduleClear();

	/* 收尾：CE 拉低，关掉 EXTI，别让中断再进来 */
	px4_arch_gpiowrite(NRF24L01_CE_GPIO, 0);
	px4_arch_gpiosetevent(NRF24L01_IRQ_GPIO, false, false, false, nullptr, nullptr);

	ModuleBase::request_stop();
}

/* ==========================================================================
 * ModuleBase：启动 / 自定义命令 / 用法
 * ========================================================================== */
int NRF24L01::task_spawn(int argc, char *argv[])
{
	NRF24L01 *instance = new NRF24L01();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue; /* 我没有自己的线程，跑在工作队列上 */

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

/* `nrf24l01 send ...` / `nrf24l01 regs` 从这里进来 */
int NRF24L01::custom_command(int argc, char *argv[])
{
	const char *command = (argc > 0) ? argv[0] : nullptr;

	if (command == nullptr) {
		return print_usage("缺少子命令");
	}

	NRF24L01 *instance = get_instance<NRF24L01>(desc);

	if (!is_running(desc) || instance == nullptr) {
		return print_usage("模块没在运行，先 nrf24l01 start");
	}

	/* nrf24l01 regs —— 读关键寄存器 */
	if (!strcmp(command, "regs")) {
		return instance->dump_registers();
	}

	/* nrf24l01 send test —— 发一包标准测试载荷（seq 自增） */
	if (!strcmp(command, "send")) {
		if (argc == 2 && !strcmp(argv[1], "test")) {
			nrf24l01_test_payload_s payload {};
			payload.seq = ++instance->_test_seq;
			payload.v1 = 1.0f + (float)(payload.seq % 100) * 0.01f;
			payload.v2 = -payload.v1;
			payload.v3 = 3.14f;

			const int ret = instance->send_payload((const uint8_t *)&payload, sizeof(payload));
			PX4_INFO("发送测试包 seq=%lu: %s", (unsigned long)payload.seq, ret == PX4_OK ? "收到 ACK(对端收到)" : "失败/无 ACK");
			return ret;
		}

		/* nrf24l01 send 01 02 03 ... —— 十六进制字节，最多 32 个 */
		uint8_t buffer[NRF24L01_PAYLOAD_MAX] {};
		int len = 0;

		for (int i = 1; i < argc && len < NRF24L01_PAYLOAD_MAX; i++) {
			char *endptr = nullptr;
			const unsigned long value = strtoul(argv[i], &endptr, 16);

			if (endptr == argv[i] || value > 0xFF) {
				return print_usage("send 后面只能是 1~32 个十六进制字节，例如: send 01 02 0a");
			}

			buffer[len++] = (uint8_t)value;
		}

		if (len == 0) {
			return print_usage("send 后面没有字节");
		}

		const int ret = instance->send_payload(buffer, (uint8_t)len);
		PX4_INFO("发送 %d 字节: %s", len, ret == PX4_OK ? "收到 ACK(对端收到)" : "失败/无 ACK");
		return ret;
	}

	return print_usage("未知命令");
}

int NRF24L01::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
nRF24L01 2.4GHz 无线模块驱动（rm2027/fmu-h743）。

挂在硬件 SPI2 上（临时借用 H1/H2 排针座，见 NRF24L01_pins.h），
开机即进入常驻接收模式，收到包会打印；也可以用 `send` 主动发一包。

射频参数（信道 2 / 2Mbps / 5 字节地址 11:22:33:44:55 / 2 字节 CRC / 动态包长）
与对端 F103 测试板必须完全一致，详见 src/drivers/wireless/nrf24l01/。

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("nrf24l01", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "stop the driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "print status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("regs", "dump the nRF24L01 registers (debug)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("send test", "send one 16-byte test packet");
	PRINT_MODULE_USAGE_COMMAND_DESCR("send <hex bytes>", "send 1..32 raw bytes, e.g. send 01 02 03");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return PX4_OK;
}
