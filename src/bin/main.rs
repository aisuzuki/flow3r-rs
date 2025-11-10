#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use bt_hci::controller::ExternalController;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::ChannelIFace,
        timer::{Timer as LedcTimer, TimerIFace},
    },
    //    gpio::Io,
    rmt::Rmt,
    spi::{
        Mode,
        master::{Config, Spi},
    },
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_hal_smartled::{SmartLedsAdapter, smart_led_buffer};
use esp_radio::ble::controller::BleConnector;
use smart_leds::{
    RGB8, SmartLedsWrite, brightness, gamma,
    hsv::{Hsv, hsv2rgb},
};
use trouble_host::prelude::*;

use gc9a01::prelude::*;
use gc9a01::{Gc9a01, prelude::*};

extern crate alloc;

// mod rw_interface;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.0.1

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 73744);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(size: 64 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    let (mut _wifi_controller, _interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");
    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport = BleConnector::new(&radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 20>::new(transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let _stack = trouble_host::new(ble_controller, &mut resources);

    // TODO: Spawn some tasks
    let _ = spawner;

    // ===== LEDC baclight ====
    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut lstimer0: LedcTimer<'_, LowSpeed> =
        ledc.timer::<LowSpeed>(esp_hal::ledc::timer::Number::Timer0);
    lstimer0
        .configure(esp_hal::ledc::timer::config::Config {
            duty: esp_hal::ledc::timer::config::Duty::Duty5Bit,
            clock_source: esp_hal::ledc::timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(24),
        })
        .expect("Failed to configure LEDC timer");

    let mut backlight = ledc.channel(esp_hal::ledc::channel::Number::Channel0, peripherals.GPIO46);
    backlight
        .configure(esp_hal::ledc::channel::config::Config {
            timer: &lstimer0,
            duty_pct: 10,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("Failed to configure LEDC channel");

    backlight.set_duty(80).expect("Failed to set backlight");

    // ===== Display test =====
    let dma_channel = peripherals.DMA_CH0;
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);

    let dma_rx_buf =
        DmaRxBuf::new(rx_descriptors, rx_buffer).expect("Failed to create DMA RX buffer");

    let dma_tx_buf =
        DmaTxBuf::new(tx_descriptors, tx_buffer).expect("Failed to create DMA TX buffer");
    // todo: with 80mhz, Mode0
    let mut spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_khz(80))
            .with_mode(Mode::_0),
    )
    .expect("Failed to create SPI")
    .with_sck(peripherals.GPIO41)
    .with_mosi(peripherals.GPIO42)
    .with_dma(dma_channel)
    .with_buffers(dma_rx_buf, dma_tx_buf);

    //    Gc9a01::new(None, gc9a01::Size240x240);

    // =====  LED test =====
    // Configure RMT (Remote Control Transceiver) peripheral globally
    // <https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/api-reference/peripherals/rmt.html>
    let rmt: Rmt<'_, esp_hal::Blocking> =
        Rmt::new(peripherals.RMT, Rate::from_mhz(80)).expect("Failed to initialize RMT");

    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    let rmt_channel = rmt.channel0;
    let rmt_buffer = smart_led_buffer!(40);

    // Each devkit uses a unique GPIO for the RGB LED, so in order to support
    // all chips we must unfortunately use `#[cfg]`s:
    let mut led: SmartLedsAdapter<'_, 961> =
        SmartLedsAdapter::new(rmt_channel, peripherals.GPIO14, rmt_buffer);

    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    let mut data: RGB8;
    let level = 10;

    loop {
        // Iterate over the rainbow!
        for hue in 0..=255 {
            color.hue = hue;
            // Convert from the HSV color space (where we can easily transition from one
            // color to the other) to the RGB color space that we can then send to the LED
            data = hsv2rgb(color);
            // When sending to the LED, we do a gamma correction first (see smart_leds docs
            // for details <https://docs.rs/smart-leds/latest/smart_leds/struct.Gamma.html>)
            // and then limit the brightness level to 10 out of 255 so that the output
            // is not too bright.
            led.write(brightness(gamma([data; 40].into_iter()), level))
                .unwrap();
            Timer::after(Duration::from_millis(20)).await;
            //            esp_println::println!("light!");
        }
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples/src/bin
}
