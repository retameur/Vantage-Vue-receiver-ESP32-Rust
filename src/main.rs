use std::fmt::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

use esp_idf_svc::hal::delay::TickType;
use esp_idf_svc::hal::uart;
use esp_idf_svc::hal::{
    peripherals::Peripherals,
    gpio,
    gpio::PinDriver,
    spi, spi::*,
    i2c::{ I2cConfig, I2cDriver },
    units::Hertz,
    delay,
    uart::*
};
use::esp_idf_svc::timer::EspTimerService;

use bmp280_ehal::{ BMP280, Config, Control };
use aht20_driver::AHT20;
use rfm69::{ Rfm69, registers, registers::Mode };
use crc16::{ State, KERMIT };

static RFM69_IRQ_FLAG: AtomicBool = AtomicBool::new(false);

fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    log::info!("Hello from receiver log interface");

    let peripherals = Peripherals::take()?;
    let pins = peripherals.pins;



    // Init UART
    // Console messages are sent on UART1, so we can plug a Raspberry Pi with WeeWX and the MeteoStick driver for example.
    
    let tx = pins.gpio1;
    let rx = pins.gpio2;

    let config = uart::config::Config::new().baudrate(Hertz(115_200));
    let mut uart = UartDriver::new(
        peripherals.uart1,
        tx,
        rx,
        Option::<gpio::Gpio0>::None,
        Option::<gpio::Gpio1>::None,
        &config,
    )
    .unwrap();



    // BMP280 & ATH20 Module ---------------

    let i2c_config = I2cConfig::new().scl_enable_pullup(true).sda_enable_pullup(true).baudrate(Hertz(100_000));

    let sda = pins.gpio14;
    let scl = pins.gpio21;
    let i2c_driver = I2cDriver::new(peripherals.i2c1, sda, scl, &i2c_config)?;
    
    // As the I2C bus is used by two sensors (on the same module), we use the shared_bus crate.
    let i2c_bus = shared_bus::BusManagerSimple::new(i2c_driver);
    
    let mut bmp = BMP280::new_with_address(i2c_bus.acquire_i2c(), 0x77)?;
    bmp.set_config(Config{
        t_sb: bmp280_ehal::Standby::ms250,
        filter: bmp280_ehal::Filter::c16
    });
    bmp.set_control(Control{
        osrs_t: bmp280_ehal::Oversampling::x2,
        osrs_p: bmp280_ehal::Oversampling::x16,
        mode: bmp280_ehal::PowerMode::Normal
    });
    log::info!("BPM280 initialized -> id: {:?}", bmp.id());

    let mut delay = delay::Delay::new_default();
    let mut ath20_uninit = AHT20::new(i2c_bus.acquire_i2c(), 0x38);
    let mut ath20 = ath20_uninit.init(&mut delay).unwrap();

    log::info!("ATH20 initialized");
    

    // RFM69 -------------------------------
    
    let pin_rfm69_irq = pins.gpio8; // DI0 on RFM69
    let pin_sclk = pins.gpio7; // SCK on RFM69
    let pin_cs = pins.gpio6; // NSS on RFM69
    let pin_sdo = pins.gpio15; // MOSI on RFM69
    let pin_sdi = Some(pins.gpio16); // MISO on RFM69
    
    // Subscribe to RFM69 new message interrupt
    let mut rfm69_irq = PinDriver::input(pin_rfm69_irq)?;
    rfm69_irq.set_interrupt_type(gpio::InterruptType::PosEdge)?;
    unsafe { 
        rfm69_irq.subscribe(rfm69_irq_callback)?;
    }

    // Instantiate SPI Driver
    let spi = peripherals.spi2;
    let spi_drv = SpiDriver::new(
        spi,
        pin_sclk,
        pin_sdo,
        pin_sdi,
        &SpiDriverConfig::default(),
    ).unwrap();
    
    // Configure Parameters for SPI device
    let spi_config = spi::config::Config::new()
        .bit_order(spi::config::BitOrder::MsbFirst)
        .data_mode(spi::config::MODE_0)
        .baudrate(Hertz(1_000_000));
    
    // Instantiate SPI Device Driver and Pass Configuration
    let spi = SpiDeviceDriver::new(spi_drv, Some(pin_cs), &spi_config).unwrap();
    let mut rfm69 = Rfm69::new(spi);
    
    // Set RFM69 registers for proper configuration
    config_rfm69(&mut rfm69);


    let mut channel = 0;
    // European frequencies
    let frequencies = [
        868_066_711,
        868_297_119,
        868_527_466,
        868_181_885,
        868_412_292
    ];
    rfm69.frequency(frequencies[channel]).unwrap();

    // Read and print all RFM69 registers
    let all_regs = rfm69.read_all_regs().unwrap();
    log::info!("RFM69 registers: {all_regs:02X?}");

    rfm69_irq.enable_interrupt()?;
    rfm69.mode(Mode::Receiver).unwrap();
    while !rfm69.is_mode_ready().unwrap() {
        thread::sleep(Duration::from_millis(100));
    }

    log::info!("RFM69 initialized");



    // Main loop ---------------------------------
    
    let timer_service = EspTimerService::new().unwrap();
    let mut buffer = [0; 10];
    let mut last_rx: u128 = 0;

    loop {
        if RFM69_IRQ_FLAG.load(Ordering::Relaxed) {
            RFM69_IRQ_FLAG.store(false, Ordering::Relaxed);

            rfm69.mode(Mode::Receiver).unwrap();
            if rfm69.is_packet_ready().unwrap() {

                // This call will not block.
                rfm69.recv(&mut buffer).unwrap();

                let rssi = rfm69.rssi();
                
                let buf_len = buffer.len();
                if buf_len != 10 {
                    log::info!("Invalid message: len: {buf_len:?}");
                    rfm69.mode(Mode::Receiver).unwrap();
                    rfm69_irq.enable_interrupt()?;
                    continue;
                }

                let crc16 = State::<KERMIT>::calculate(&buffer[0..=7]);
                if crc16 != 0 || buffer[8] != 0xFF || buffer[9] != 0xFF {
                    let b9 = buffer[8];
                    let b10 = buffer[9];
                    log::info!("Invalid message: crc16: {crc16:?}, b9: {b9:02X}, b10: {b10:02X}, rssi: {rssi:?}");
                    rfm69.mode(Mode::Receiver).unwrap();
                    rfm69_irq.enable_interrupt()?;
                    continue;
                }

                // Send I packet based on RFM69 data
                let now_micros = timer_service.now().as_micros();
                if last_rx == 0 { last_rx = now_micros; }
                let delta = now_micros - last_rx;
                last_rx = now_micros;

                log::info!("Raw message: {buffer:02X?}. RSSI: {rssi:?} CRC: {crc16:?}");

                let buffer_msb = buffer.map(|lsb| {
                    let msb = reverse_bits(lsb);
                    format!("{msb:02X}")
                });
                let buffer_hex_str = buffer_msb.join(" ");

                // The extra space between buffer_hex_str and rssi is needed.
                let i_message = format!("I 10{channel:?} {buffer_hex_str}  {rssi:?} {delta:?} 0\n");
                uart.write_str(&i_message).ok();
                log::info!("Sent to UART1: {i_message}");

                // Send B packet based on BMP and ATH data
                // The temperature is available on both the BPM and the ATH. Choose either or average them.
                let bmp_temp = bmp.temp();
                let bmp_pressure = bmp.pressure();
                let bmp_pressure_rounded = bmp_pressure.round() as u32;
                let ath_measure = ath20.measure(&mut delay).unwrap();
                let ath_temp_rounded = (ath_measure.temperature * 10.0).round() as u16;
                let ath_humidity_rounded = ath_measure.humidity.round() as u16;
                log::info!("BMP -> temp: {bmp_temp:?}, pressure: {bmp_pressure:?} ATH -> {ath_measure:?}");
                let b_message = format!("B 0 0 {ath_temp_rounded:?} {bmp_pressure_rounded:?} 0 0 {ath_humidity_rounded:?}\n");
                uart.write_str(&b_message).ok();
                log::info!("Sent to UART1: {b_message}");

                // Jump to next channel
                channel += 1;
                if channel >= 5 { channel = 0; }
                rfm69.frequency(frequencies[channel]).unwrap();
            } else {
                // Should never happen
                log::info!("Packet wasn't ready");
            }
            rfm69.mode(Mode::Receiver).unwrap();
            rfm69_irq.enable_interrupt()?;
        }
        
        
        // A very crude system to make the WeeWx MeteoStick driver happy
        let mut buf = [0_u8; 1];
        match uart.read(&mut buf, TickType::new_millis(10).ticks()) {
            Ok(len) => {
                if len <= 0 { continue; }
                log::info!("UART received byte: {}", buf[0]);
                    match buf[0] {
                        b'r' => uart.write_str("# ESP32 Vantage Vue receiver ready to accept commands\n?"),
                        b'm' | b't' | b'f' | b'o' | b'x' => uart.write_str("# OK\n"),
                        _ => uart.write_str("# ERR\n")
                    }.ok();
            }
            Err(e) => {
                log::error!("UART read error {}", e);
            }
        }
        thread::sleep(Duration::from_millis(10));
    }
}

fn reverse_bits(byte: u8) -> u8 {
    let mut b = byte;
    b = ((b & 0b11110000) >> 4 ) | ((b & 0b00001111) << 4);
    b = ((b & 0b11001100) >> 2 ) | ((b & 0b00110011) << 2);
    ((b & 0b10101010) >>1 ) | ((b & 0b01010101) << 1)
}

fn rfm69_irq_callback() {
    RFM69_IRQ_FLAG.store(true, Ordering::Relaxed);
}

fn config_rfm69<S: rfm69::ReadWrite>(rfm: &mut Rfm69<S>) {
    use registers::*;

    let _ = rfm.dio_mapping(DioMapping {
        pin: DioPin::Dio0,
        dio_type: DioType::Dio01,
        dio_mode: DioMode::Rx
    });

    let _ = rfm.mode(Mode::Standby);

    let _ = rfm.modulation(Modulation {
        data_mode: DataMode::Packet,
        modulation_type: ModulationType::Fsk,
        shaping: ModulationShaping::Shaping10,
    });
    let _ = rfm.bit_rate(19_200);
    let _ = rfm.fdev(9_900);
    let _ = rfm.write(Registers::PaRamp, 0x0B); //RF_PARAMP_25
    let _ = rfm.lna(LnaConfig {
        zin: LnaImpedance::Ohm50,
        gain_select: LnaGain::AgcLoop
    });
    let _ = rfm.rx_bw(RxBw {
        dcc_cutoff: DccCutoff::Percent4,
        rx_bw: RxBwFsk::Khz25dot0
    });
    let _ = rfm.rx_afc_bw(RxBw { 
        dcc_cutoff: DccCutoff::Percent4, 
        rx_bw: RxBwFsk::Khz50dot0 
    });
    let _ = rfm.write(Registers::AfcFei, 0x08 | 0x04);
    let _ = rfm.write(Registers::IrqFlags2, 0x10);
    
    // The recommanded value in the documentation is 0xE4 (d228) which correspond to -114 dBm
    // In my case, this is too high and the RFM pick-up too much noise and generate mostly garbage packets.
    let _ = rfm.rssi_threshold(160); 
    
    let _ = rfm.preamble(4);
    let _ = rfm.sync(&[0xCB, 0x89]);
    let _ = rfm.write(Registers::SyncConfig, 0x8A);
    let _ = rfm.packet(PacketConfig {
        format: PacketFormat::Fixed(10),
        dc: PacketDc::None,
        filtering: PacketFiltering::None,
        crc: false,
        interpacket_rx_delay: InterPacketRxDelay::Delay2048Bits,
        auto_rx_restart: false,
    });

    let _ = rfm.aes(&[]);
    let _ = rfm.write(Registers::FifoThresh, 0x00 | 0x07);
    let _ = rfm.write(Registers::TestDagc, 0x30);
}