#include "peripheral/qdec.hpp"
#include "protocol/message_handler.hpp"

static constexpr float TWO_PI = 6.2831853f;
static constexpr float DT_S   = static_cast<float>(Qdec::SAMPLE_PERIOD_US) / 1000000.0f;

Qdec::Qdec(const struct device* dev, uint8_t idx)
    : dev_(dev)
    , idx_(idx) { }

int Qdec::init() {
    if (!device_is_ready(dev_)) {
        return -EIO;
    }

    // Read initial count
    if (sensor_sample_fetch(dev_) < 0) {
        return -EIO;
    }
    struct sensor_value val;
    if (sensor_channel_get(dev_, SENSOR_CHAN_ENCODER_COUNT, &val) < 0) return -EIO;
    last_count_ = val.val1;

    // 1000Hz timer ISR
    k_timer_init(&timer_, timer_callback, nullptr);
    timer_.user_data = this;
    k_timer_start(&timer_, K_USEC(SAMPLE_PERIOD_US), K_USEC(SAMPLE_PERIOD_US));

    return 0;
}

void Qdec::timer_callback(struct k_timer* timer) {
    static_cast<Qdec*>(timer->user_data)->on_timer();
}

void Qdec::on_timer() {
    if (sensor_sample_fetch(dev_) < 0) return;
    struct sensor_value val;
    if (sensor_channel_get(dev_, SENSOR_CHAN_ENCODER_COUNT, &val) < 0) return;

    int32_t count = val.val1;

    // Delta as int32_t — handles 16-bit timer wrap correctly
    // e.g. last=65530, current=5 → delta = 5 - 65530 = -65525 (correct)
    int32_t delta = count - last_count_;
    last_count_   = count;

    // Convert to rad/s
    // counts_per_rev = lines_per_rev * 4 (x4 mode)
    // rad/s = (delta / counts_per_rev) * 2π / dt
    if (configured_ && counts_per_rev_ > 0) {
        velocity_ = static_cast<float>(delta) * TWO_PI / static_cast<float>(counts_per_rev_) / DT_S;

        msg::encode_encoder(uplink_writer_, idx_, velocity_);
    }
}
