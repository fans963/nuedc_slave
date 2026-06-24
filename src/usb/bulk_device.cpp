#include "usb/bulk_device.hpp"
#include "peripheral/adc.hpp"
#include "peripheral/can.hpp"
#include "peripheral/i2c.hpp"
#include "peripheral/imu.hpp"
#include "peripheral/motor.hpp"
#include "peripheral/qdec.hpp"
#include "peripheral/spi.hpp"
#include "peripheral/uart.hpp"
#include <cstring>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usbd.h>

// ── USB Descriptors ────────────────────────────────────────────────────

static constexpr uint8_t BULK_OUT_EP = 0x01;
static constexpr uint8_t BULK_IN_EP  = 0x81;

struct bulk_desc {
    struct usb_if_descriptor if0;
    struct usb_ep_descriptor if0_out_ep;
    struct usb_ep_descriptor if0_in_ep;
    struct usb_ep_descriptor if0_hs_out_ep;
    struct usb_ep_descriptor if0_hs_in_ep;
    struct usb_desc_header nil_desc;
};

static struct bulk_desc desc = {
    .if0 =
        {
            .bLength = sizeof(struct usb_if_descriptor),
            .bDescriptorType = USB_DESC_INTERFACE,
            .bInterfaceNumber = 0,
            .bAlternateSetting = 0,
            .bNumEndpoints = 2,
            .bInterfaceClass = USB_BCC_VENDOR,
            .bInterfaceSubClass = 0,
            .bInterfaceProtocol = 0,
            .iInterface = 0,
        },
    .if0_out_ep =
        {
            .bLength = sizeof(struct usb_ep_descriptor),
            .bDescriptorType = USB_DESC_ENDPOINT,
            .bEndpointAddress = BULK_OUT_EP,
            .bmAttributes = USB_EP_TYPE_BULK,
            .wMaxPacketSize = sys_cpu_to_le16(64),
            .bInterval = 0,
        },
    .if0_in_ep =
        {
            .bLength = sizeof(struct usb_ep_descriptor),
            .bDescriptorType = USB_DESC_ENDPOINT,
            .bEndpointAddress = BULK_IN_EP,
            .bmAttributes = USB_EP_TYPE_BULK,
            .wMaxPacketSize = sys_cpu_to_le16(64),
            .bInterval = 0,
        },
    .if0_hs_out_ep =
        {
            .bLength = sizeof(struct usb_ep_descriptor),
            .bDescriptorType = USB_DESC_ENDPOINT,
            .bEndpointAddress = BULK_OUT_EP,
            .bmAttributes = USB_EP_TYPE_BULK,
            .wMaxPacketSize = sys_cpu_to_le16(512),
            .bInterval = 0,
        },
    .if0_hs_in_ep =
        {
            .bLength = sizeof(struct usb_ep_descriptor),
            .bDescriptorType = USB_DESC_ENDPOINT,
            .bEndpointAddress = BULK_IN_EP,
            .bmAttributes = USB_EP_TYPE_BULK,
            .wMaxPacketSize = sys_cpu_to_le16(512),
            .bInterval = 0,
        },
    .nil_desc = {0, 0},
};

static const struct usb_desc_header* fs_descs[] = {
    (struct usb_desc_header*)&desc.if0,
    (struct usb_desc_header*)&desc.if0_out_ep,
    (struct usb_desc_header*)&desc.if0_in_ep,
    (struct usb_desc_header*)&desc.nil_desc,
};

static const struct usb_desc_header* hs_descs[] = {
    (struct usb_desc_header*)&desc.if0,
    (struct usb_desc_header*)&desc.if0_hs_out_ep,
    (struct usb_desc_header*)&desc.if0_hs_in_ep,
    (struct usb_desc_header*)&desc.nil_desc,
};

// ── USBD context and descriptors ───────────────────────────────────────

USBD_DEVICE_DEFINE(usbd_ctx, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), 0x1209, 0x0963);

USBD_DESC_LANG_DEFINE(usbd_lang);
USBD_DESC_MANUFACTURER_DEFINE(usbd_mfr, "NUEDC2026");
USBD_DESC_PRODUCT_DEFINE(usbd_product, "NUEDC Slave");
USBD_DESC_CONFIG_DEFINE(usbd_fs_cfg, "FS Configuration");
USBD_DESC_CONFIG_DEFINE(usbd_hs_cfg, "HS Configuration");

USBD_CONFIGURATION_DEFINE(usbd_fs_config, 0, 100, &usbd_fs_cfg);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
USBD_CONFIGURATION_DEFINE(usbd_hs_config, 0, 100, &usbd_hs_cfg);
#pragma GCC diagnostic pop

// ── Buffer pool ────────────────────────────────────────────────────────

NET_BUF_POOL_FIXED_DEFINE(bulk_pool, 4, 512, sizeof(struct udc_buf_info), NULL);

// ── Global state ───────────────────────────────────────────────────────

static BulkDevice* g_bulk               = nullptr;
static struct usbd_class_data* g_c_data = nullptr;
static atomic_t g_enabled               = ATOMIC_INIT(0);

// ── Class API ──────────────────────────────────────────────────────────

static struct bulk_func_data {
    struct bulk_desc* const d               = &desc;
    const struct usb_desc_header** const fs = fs_descs;
    const struct usb_desc_header** const hs = hs_descs;
} fdata;

static int bulk_request(struct usbd_class_data* c_data, struct net_buf* buf, int err) {
    struct udc_buf_info* bi = udc_get_buf_info(buf);

    if (err) {
        net_buf_unref(buf);
        return 0;
    }

    if (bi->ep == BULK_OUT_EP) {
        if (g_bulk) g_bulk->on_rx_data(buf->data, buf->len);
        net_buf_unref(buf);
        struct net_buf* nb = net_buf_alloc(&bulk_pool, K_NO_WAIT);
        if (nb) {
            udc_get_buf_info(nb)->ep = BULK_OUT_EP;
            usbd_ep_enqueue(c_data, nb);
        }
    } else {
        net_buf_unref(buf);
        if (g_bulk) g_bulk->on_tx_done();
    }
    return 0;
}

static int bulk_init(struct usbd_class_data* c_data) {
    ARG_UNUSED(c_data);
    return 0;
}

static void bulk_enable(struct usbd_class_data* c_data) {
    atomic_set(&g_enabled, 1);
    g_c_data = c_data;

    struct net_buf* buf = net_buf_alloc(&bulk_pool, K_NO_WAIT);
    if (buf) {
        udc_get_buf_info(buf)->ep = BULK_OUT_EP;
        usbd_ep_enqueue(c_data, buf);
    }
}

static void bulk_disable(struct usbd_class_data* c_data) { atomic_set(&g_enabled, 0); }

static void* bulk_get_desc(struct usbd_class_data* c_data, enum usbd_speed speed) {
    return (void*)((speed == USBD_SPEED_HS) ? hs_descs : fs_descs);
}

static const struct usbd_class_api bulk_api = {
    .request  = bulk_request,
    .enable   = bulk_enable,
    .disable  = bulk_disable,
    .init     = bulk_init,
    .get_desc = bulk_get_desc,
};

USBD_DEFINE_CLASS(bulk_class, &bulk_api, &fdata, NULL);

// ── BulkDevice implementation ──────────────────────────────────────────

BulkDevice::BulkDevice()
    : decoder_(*this) { }

int BulkDevice::init() {
    g_bulk = this;

    /* Order: descriptors → config → class → init → enable */
    usbd_add_descriptor(&usbd_ctx, &usbd_lang);
    usbd_add_descriptor(&usbd_ctx, &usbd_mfr);
    usbd_add_descriptor(&usbd_ctx, &usbd_product);

    int ret = usbd_add_configuration(&usbd_ctx, USBD_SPEED_FS, &usbd_fs_config);
    if (ret < 0) return ret;

    ret = usbd_register_class(&usbd_ctx, "bulk_class", USBD_SPEED_FS, 1);
    if (ret < 0) return ret;

    ret = usbd_init(&usbd_ctx);
    if (ret < 0) return ret;

    ret = usbd_enable(&usbd_ctx);
    if (ret < 0) return ret;

    return 0;
}

void BulkDevice::set_cans(std::span<Can* const> cans) {
    for (size_t i = 0; i < cans.size() && i < MAX_CAN; ++i)
        cans_[i] = cans[i];
}

void BulkDevice::set_uarts(std::span<Uart* const> uarts) {
    for (size_t i = 0; i < uarts.size() && i < MAX_UART; ++i)
        uarts_[i] = uarts[i];
}

void BulkDevice::set_spis(std::span<Spi* const> spis) {
    for (size_t i = 0; i < spis.size() && i < MAX_SPI; ++i)
        spis_[i] = spis[i];
}

void BulkDevice::set_i2cs(std::span<I2c* const> i2cs) {
    for (size_t i = 0; i < i2cs.size() && i < MAX_I2C; ++i)
        i2cs_[i] = i2cs[i];
}

void BulkDevice::set_imus(std::span<IMU* const> imus) {
    for (size_t i = 0; i < imus.size() && i < MAX_IMU; ++i)
        imus_[i] = imus[i];
}

void BulkDevice::set_adcs(std::span<Adc* const> adcs) {
    for (size_t i = 0; i < adcs.size() && i < MAX_ADC; ++i)
        adcs_[i] = adcs[i];
}

void BulkDevice::set_qdecs(std::span<Qdec* const> qdecs) {
    for (size_t i = 0; i < qdecs.size() && i < MAX_QDEC; ++i)
        qdecs_[i] = qdecs[i];
}

void BulkDevice::set_motors(std::span<Motor* const> motors) {
    for (size_t i = 0; i < motors.size() && i < MAX_MOTOR; ++i)
        motors_[i] = motors[i];
}

void BulkDevice::on_rx_data(const uint8_t* data, size_t len) { decoder_.feed(data, len); }

void BulkDevice::on_tx_done() { atomic_set(&tx_busy_, 0); }

void BulkDevice::swap_tx_buffers() {
    auto* tmp    = tx_fill_buf_;
    tx_fill_buf_ = tx_send_buf_;
    tx_send_buf_ = tmp;
}

void BulkDevice::try_transmit() {
    if (!atomic_get(&g_enabled) || !g_c_data || atomic_get(&tx_busy_)) return;

    // Collect uplink from all registered peripherals
    auto collect_writer = [this](auto& writer) {
        if (tx_fill_len_ >= TX_BUF_SIZE) return;
        writer.drain([this](std::span<const uint8_t> chunk) {
            size_t space = TX_BUF_SIZE - tx_fill_len_;
            size_t n     = std::min(chunk.size(), space);
            std::memcpy(tx_fill_buf_ + tx_fill_len_, chunk.data(), n);
            tx_fill_len_ += n;
        });
    };

    for (auto* c : cans_)
        if (c) collect_writer(c->uplink_writer());
    for (auto* u : uarts_)
        if (u) collect_writer(u->uplink_writer());
    for (auto* s : spis_)
        if (s) collect_writer(s->uplink_writer());
    for (auto* i : i2cs_)
        if (i) collect_writer(i->uplink_writer());
    for (auto* m : imus_)
        if (m) collect_writer(m->uplink_writer());
    for (auto* a : adcs_)
        if (a) collect_writer(a->uplink_writer());
    for (auto* q : qdecs_)
        if (q) collect_writer(q->uplink_writer());

    if (tx_fill_len_ == 0) return;

    swap_tx_buffers();
    tx_send_len_ = tx_fill_len_;
    tx_fill_len_ = 0;

    struct net_buf* buf = net_buf_alloc(&bulk_pool, K_NO_WAIT);
    if (!buf) {
        tx_fill_len_ = tx_send_len_;
        swap_tx_buffers();
        return;
    }

    std::memcpy(net_buf_add(buf, tx_send_len_), tx_send_buf_, tx_send_len_);
    udc_get_buf_info(buf)->ep = BULK_IN_EP;

    atomic_set(&tx_busy_, 1);
    if (usbd_ep_enqueue(g_c_data, buf) < 0) {
        net_buf_unref(buf);
        atomic_set(&tx_busy_, 0);
    }
}

void BulkDevice::on_frame(const uint8_t* data, size_t len) {
    auto* c0 = cans_[0];
    auto* u0 = uarts_[0];
    if (!c0 || !u0) return;

    msg::dispatch_downlink(
        data, len, c0->downlink_buf(), cans_[1] ? cans_[1]->downlink_buf() : c0->downlink_buf(),
        u0->downlink_buf(), uarts_[1] ? uarts_[1]->downlink_buf() : u0->downlink_buf(),
        // Encoder config callback
        [this](uint8_t id, uint16_t lines_per_rev) {
            if (id < qdecs_.size() && qdecs_[id]) {
                qdecs_[id]->set_lines_per_rev(lines_per_rev);
            }
        },
        // Motor command callback (target_speed in rad/s)
        [this](uint8_t id, float target_speed) {
            if (id < motors_.size() && motors_[id]) {
                motors_[id]->set_target(target_speed);
            }
        },
        // PID config callback
        [this](uint8_t id, float kp, float ki, float kd, float out_min, float out_max,
            float int_min, float int_max, bool reset) {
            if (id < motors_.size() && motors_[id]) {
                motors_[id]->set_pid(kp, ki, kd);
                motors_[id]->set_limits(out_min, out_max, int_min, int_max);
                if (reset) {
                    motors_[id]->set_target(0.0f);
                }
            }
        });
}
