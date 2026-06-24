#include "util/log.hpp"
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "peripheral/adc.hpp"
#include "peripheral/can.hpp"
#include "peripheral/i2c.hpp"
#include "peripheral/imu.hpp"
#include "peripheral/motor.hpp"
#include "peripheral/qdec.hpp"
#include "peripheral/spi.hpp"
#include "peripheral/status_led.hpp"
#include "peripheral/uart.hpp"
#include "usb/bulk_device.hpp"

static constexpr auto LOG = util::log::Logger { "main" };

// ── Peripheral instances (all via DT aliases) ──────────────────────────

// UART
static Uart uart0(DEVICE_DT_GET(DT_ALIAS(zephyr_uart1)), 0);
static Uart uart1(DEVICE_DT_GET(DT_ALIAS(zephyr_uart2)), 1);

// CAN
static Can can0(DEVICE_DT_GET(DT_ALIAS(zephyr_can1)), 0);

// IMU (BMI088 accel + gyro)
static IMU imu0(
    DEVICE_DT_GET(DT_ALIAS(zephyr_imu0_accel)), DEVICE_DT_GET(DT_ALIAS(zephyr_imu0_gyro)));

// SPI (raw, generic)
static Spi spi1(DEVICE_DT_GET(DT_ALIAS(zephyr_spi2)), 1);

// I2C
static I2c i2c0(DEVICE_DT_GET(DT_ALIAS(zephyr_i2c1)), 0);
static I2c i2c1(DEVICE_DT_GET(DT_ALIAS(zephyr_i2c2)), 1);

// ADC
static Adc adc0(DEVICE_DT_GET(DT_ALIAS(zephyr_adc1)), 0);
static Adc adc1(DEVICE_DT_GET(DT_ALIAS(zephyr_adc2)), 1);

// Quadrature encoders (1000Hz ISR-driven)
static Qdec qdec0(DEVICE_DT_GET(DT_ALIAS(zephyr_qdec0)), 0);
static Qdec qdec1(DEVICE_DT_GET(DT_ALIAS(zephyr_qdec1)), 1);

// Motors (PID velocity control, 1000Hz ISR-driven)
static constexpr Motor::Config MOTOR_CFG = {
    .pwm_period_ns = 50000, // 20kHz
    .output_min    = -1.0f,
    .output_max    = 1.0f,
    .vel_min       = -100.0f, // -100 rad/s
    .vel_max       = 100.0f,  // +100 rad/s
};
static Motor motor0(DEVICE_DT_GET(DT_ALIAS(zephyr_pwm_motor)), 0, qdec0, MOTOR_CFG);
static Motor motor1(DEVICE_DT_GET(DT_ALIAS(zephyr_pwm_motor)), 1, qdec1, MOTOR_CFG);

// USB bulk device
static BulkDevice bulk_dev;

// Status LED
static StatusLed status_led;

/// Check if any uplink ring buffer has data that isn't being drained.
static bool any_ringbuf_full() {
    static constexpr size_t THRESHOLD = 400;

    return can0.uplink_writer().readable() > THRESHOLD
        || uart0.uplink_writer().readable() > THRESHOLD
        || uart1.uplink_writer().readable() > THRESHOLD
        || imu0.uplink_writer().readable() > THRESHOLD
        || spi1.uplink_writer().readable() > THRESHOLD
        || adc0.uplink_writer().readable() > THRESHOLD
        || adc1.uplink_writer().readable() > THRESHOLD
        || qdec0.uplink_writer().readable() > THRESHOLD
        || qdec1.uplink_writer().readable() > THRESHOLD;
}

int main(void) {
    // LED 蓝色 — 初始化中
    status_led.init();

    LOG.info("NUEDC slave starting");

    // Init all peripherals
    uart0.init();
    uart1.init();
    can0.init();
    imu0.init();
    spi1.init();
    i2c0.init();
    i2c1.init();
    adc0.init();
    adc1.init();

    // Encoders + Motors
    qdec0.init();
    qdec1.init();
    motor0.init();
    motor1.init();

    // Init USB — register all peripherals
    Can* cans[]     = { &can0 };
    Uart* uarts[]   = { &uart0, &uart1 };
    Spi* spis[]     = { &spi1 };
    I2c* i2cs[]     = { &i2c0, &i2c1 };
    IMU* imus[]     = { &imu0 };
    Adc* adcs[]     = { &adc0, &adc1 };
    Qdec* qdecs[]   = { &qdec0, &qdec1 };
    Motor* motors[] = { &motor0, &motor1 };

    bulk_dev.set_cans(cans);
    bulk_dev.set_uarts(uarts);
    bulk_dev.set_spis(spis);
    bulk_dev.set_i2cs(i2cs);
    bulk_dev.set_imus(imus);
    bulk_dev.set_adcs(adcs);
    bulk_dev.set_qdecs(qdecs);
    bulk_dev.set_motors(motors);
    bulk_dev.init();

    // LED 绿色 — 正常运行
    status_led.set(StatusLed::State::Running);
    LOG.info("All peripherals initialized");

    while (true) {
        // IMU is interrupt-driven, no polling needed
        // adc0.poll();
        // adc1.poll();

        // Drain uplink → USB
        bulk_dev.try_transmit();

        // Drain downlink → hardware
        can0.try_transmit();
        uart0.try_transmit();
        uart1.try_transmit();

        // LED 状态更新
        status_led.report_ringbuf_status(any_ringbuf_full());
    }

    return 0;
}
