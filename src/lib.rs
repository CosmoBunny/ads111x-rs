#![no_std]

#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c as AsyncI2c;

use embedded_hal::i2c::I2c;

pub struct ADS111x<I2C, const MODEL: u8> {
    address: Address,
    i2c: I2C,
    pub config: Config,
}

pub type ADS1115<I2C> = ADS111x<I2C, 5>;
pub type ADS1114<I2C> = ADS111x<I2C, 4>;
pub type ADS1113<I2C> = ADS111x<I2C, 3>;

#[cfg(feature = "async")]
impl<I2C> ADS1113<I2C> {
    pub fn new(i2c: I2C, config: Config) -> Self {
        Self {
            address: Address::Ground,
            i2c,
            config,
        }
    }
}

impl<I2C: I2c> ADS1113<I2C> {
    fn read_channel_raw(&mut self) -> Result<i16, I2C::Error> {
        let mut config = [(1 << 7) | 1, (self.config.data_rate as u8)];
        let mut data = [0u8; 2];

        self.i2c
            .write(self.address as u8, &[0x01, config[0], config[1]])?;

        loop {
            self.i2c
                .write_read(self.address as u8, &[0x01], &mut config)?;
            if (config[0] & 0x80) != 0 {
                break;
            }
        }

        self.i2c
            .write_read(self.address as u8, &[0x00], &mut data)?;
        Ok(i16::from_be_bytes(data))
    }

    pub fn read_adc(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_raw()
    }
}

#[cfg(feature = "async")]
impl<I2C: AsyncI2c> ADS1113<I2C> {
    async fn read_channel_asyncraw(&mut self) -> Result<i16, I2C::Error> {
        let mut config = [(1 << 7) | 1, (self.config.data_rate as u8)];
        let mut data = [0u8; 2];

        self.i2c
            .write(self.address as u8, &[0x01, config[0], config[1]])
            .await?;

        loop {
            self.i2c
                .write_read(self.address as u8, &[0x01], &mut config)
                .await?;
            if (config[0] & 0x80) != 0 {
                break;
            }
        }

        self.i2c
            .write_read(self.address as u8, &[0x00], &mut data)
            .await?;
        Ok(i16::from_be_bytes(data))
    }

    pub async fn read_asyncadc(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_asyncraw().await
    }
}

impl<I2C> ADS1114<I2C> {
    pub fn new(i2c: I2C, config: Config) -> Self {
        Self {
            address: Address::Ground,
            i2c,
            config,
        }
    }
}

impl<I2C: I2c> ADS1114<I2C> {
    /// Configures the ALERT/RDY pin as a conversion ready pin.
    /// Note: `config.comp_que` must NOT be `DisableComparator` (11b) for this to work.
    pub fn enable_conversion_ready_pin(&mut self) -> Result<(), I2C::Error> {
        // Set Hi_thresh MSB to 1 (0x8000)
        self.i2c.write(self.address as u8, &[0x03, 0x80, 0x00])?;
        // Set Lo_thresh MSB to 0 (0x0000)
        self.i2c.write(self.address as u8, &[0x02, 0x00, 0x00])?;
        Ok(())
    }

    fn read_channel_raw(&mut self) -> Result<i16, I2C::Error> {
        let mut config = [
            (self.config.gain as u8) | (1 << 7) | 1,
            (self.config.comp_que as u8)
                | (self.config.comp_lat as u8)
                | (self.config.comp_pol as u8)
                | (self.config.comp_mode as u8)
                | (self.config.data_rate as u8),
        ];
        let mut data = [0u8; 2];

        self.i2c
            .write(self.address as u8, &[0x01, config[0], config[1]])?;

        loop {
            self.i2c
                .write_read(self.address as u8, &[0x01], &mut config)?;
            if (config[0] & 0x80) != 0 {
                break;
            }
        }

        self.i2c
            .write_read(self.address as u8, &[0x00], &mut data)?;
        Ok(i16::from_be_bytes(data))
    }

    pub fn read_adc(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_raw()
    }
}

#[cfg(feature = "async")]
impl<I2C: AsyncI2c> ADS1114<I2C> {
    /// Configures the ALERT/RDY pin as a conversion ready pin.
    /// Note: `config.comp_que` must NOT be `DisableComparator` (11b) for this to work.
    pub async fn enable_async_conversion_ready_pin(&mut self) -> Result<(), I2C::Error> {
        // Set Hi_thresh MSB to 1 (0x8000)
        self.i2c
            .write(self.address as u8, &[0x03, 0x80, 0x00])
            .await?;
        // Set Lo_thresh MSB to 0 (0x0000)
        self.i2c
            .write(self.address as u8, &[0x02, 0x00, 0x00])
            .await?;
        Ok(())
    }

    async fn read_channel_asyncraw(&mut self) -> Result<i16, I2C::Error> {
        let mut config = [
            (self.config.gain as u8) | (1 << 7) | 1,
            (self.config.comp_que as u8)
                | (self.config.comp_lat as u8)
                | (self.config.comp_pol as u8)
                | (self.config.comp_mode as u8)
                | (self.config.data_rate as u8),
        ];
        let mut data = [0u8; 2];

        self.i2c
            .write(self.address as u8, &[0x01, config[0], config[1]])
            .await?;

        loop {
            self.i2c
                .write_read(self.address as u8, &[0x01], &mut config)
                .await?;
            if (config[0] & 0x80) != 0 {
                break;
            }
        }

        self.i2c
            .write_read(self.address as u8, &[0x00], &mut data)
            .await?;
        Ok(i16::from_be_bytes(data))
    }

    pub async fn read_asyncadc(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_asyncraw().await
    }
}

impl<I2C> ADS1115<I2C> {
    pub fn new(address: Address, i2c: I2C, config: Config) -> Self {
        Self {
            address,
            i2c,
            config,
        }
    }
}

impl<I2C: I2c> ADS1115<I2C> {
    /// Configures the ALERT/RDY pin as a conversion ready pin.
    /// Note: `config.comp_que` must NOT be `DisableComparator` (11b) for this to work.
    pub fn enable_conversion_ready_pin(&mut self) -> Result<(), I2C::Error> {
        // Set Hi_thresh MSB to 1 (0x8000)
        self.i2c.write(self.address as u8, &[0x03, 0x80, 0x00])?;
        // Set Lo_thresh MSB to 0 (0x0000)
        self.i2c.write(self.address as u8, &[0x02, 0x00, 0x00])?;
        Ok(())
    }

    fn read_channel_raw(&mut self, mux: u8) -> Result<i16, I2C::Error> {
        let mut config = [
            (self.config.gain as u8) | (1 << 7) | (mux << 4) | 1,
            (self.config.comp_que as u8)
                | (self.config.comp_lat as u8)
                | (self.config.comp_pol as u8)
                | (self.config.comp_mode as u8)
                | (self.config.data_rate as u8),
        ];
        let mut data = [0u8; 2];

        self.i2c
            .write(self.address as u8, &[0x01, config[0], config[1]])?;

        loop {
            self.i2c
                .write_read(self.address as u8, &[0x01], &mut config)?;
            if (config[0] & 0x80) != 0 {
                break;
            }
        }

        self.i2c
            .write_read(self.address as u8, &[0x00], &mut data)?;
        Ok(i16::from_be_bytes(data))
    }

    pub fn read_adc_a0(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_raw(0b100)
    }

    pub fn read_adc_a1(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_raw(0b101)
    }

    pub fn read_adc_a2(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_raw(0b110)
    }

    pub fn read_adc_a3(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_raw(0b111)
    }

    pub fn read_adc_a0n1(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_raw(0b000)
    }

    pub fn read_adc_a0n3(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_raw(0b001)
    }

    pub fn read_adc_a1n3(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_raw(0b010)
    }

    pub fn read_adc_a2n3(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_raw(0b011)
    }

    pub fn read_4adc(&mut self) -> Result<[i16; 4], I2C::Error> {
        let mut values = [0i16; 4];
        for (i, val) in values.iter_mut().enumerate() {
            *val = self.read_channel_raw(0b100 + i as u8)?;
        }
        Ok(values)
    }
}

#[cfg(feature = "async")]
impl<I2C: AsyncI2c> ADS1115<I2C> {
    /// Configures the ALERT/RDY pin as a conversion ready pin.
    /// Note: `config.comp_que` must NOT be `DisableComparator` (11b) for this to work.
    pub async fn enable_async_conversion_ready_pin(&mut self) -> Result<(), I2C::Error> {
        // Set Hi_thresh MSB to 1 (0x8000)
        self.i2c
            .write(self.address as u8, &[0x03, 0x80, 0x00])
            .await?;
        // Set Lo_thresh MSB to 0 (0x0000)
        self.i2c
            .write(self.address as u8, &[0x02, 0x00, 0x00])
            .await?;
        Ok(())
    }

    async fn read_channel_asyncraw(&mut self, mux: u8) -> Result<i16, I2C::Error> {
        let mut config = [
            (self.config.gain as u8) | (1 << 7) | (mux << 4) | 1,
            (self.config.comp_que as u8)
                | (self.config.comp_lat as u8)
                | (self.config.comp_pol as u8)
                | (self.config.comp_mode as u8)
                | (self.config.data_rate as u8),
        ];
        let mut data = [0u8; 2];

        self.i2c
            .write(self.address as u8, &[0x01, config[0], config[1]])
            .await?;

        loop {
            self.i2c
                .write_read(self.address as u8, &[0x01], &mut config)
                .await?;
            if (config[0] & 0x80) != 0 {
                break;
            }
        }

        self.i2c
            .write_read(self.address as u8, &[0x00], &mut data)
            .await?;
        Ok(i16::from_be_bytes(data))
    }

    pub async fn read_async_adc_a0(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_asyncraw(0b100).await
    }

    pub async fn read_async_adc_a1(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_asyncraw(0b101).await
    }

    pub async fn read_async_adc_a2(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_asyncraw(0b110).await
    }

    pub async fn read_async_adc_a3(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_asyncraw(0b111).await
    }

    pub async fn read_async_adc_a0n1(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_asyncraw(0b000).await
    }

    pub async fn read_async_adc_a0n3(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_asyncraw(0b001).await
    }

    pub async fn read_async_adc_a1n3(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_asyncraw(0b010).await
    }

    pub async fn read_async_adc_a2n3(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_asyncraw(0b011).await
    }

    pub async fn read_async_4adc(&mut self) -> Result<[i16; 4], I2C::Error> {
        let mut values = [0i16; 4];
        for (i, val) in values.iter_mut().enumerate() {
            *val = self.read_channel_asyncraw(0b100 + i as u8).await?;
        }
        Ok(values)
    }
}

/**

## Config

Send the configuration to the device. But some config can't be send due to some model limitation.

*/
#[derive(Debug, Clone, Copy, Default)]
pub struct Config {
    pub gain: Gain,
    pub data_rate: DataRate,
    pub comp_mode: CompMode,
    pub comp_pol: CompPol,
    pub comp_lat: CompLat,
    pub comp_que: CompQue,
}

impl Config {
    pub fn with_gain(self, gain: Gain) -> Self {
        Self { gain, ..self }
    }

    pub fn with_data_rate(self, data_rate: DataRate) -> Self {
        Self { data_rate, ..self }
    }

    pub fn with_comp_mode(self, comp_mode: CompMode) -> Self {
        Self { comp_mode, ..self }
    }

    pub fn with_comp_pol(self, comp_pol: CompPol) -> Self {
        Self { comp_pol, ..self }
    }

    pub fn with_comp_lat(self, comp_lat: CompLat) -> Self {
        Self { comp_lat, ..self }
    }

    pub fn with_comp_que(self, comp_que: CompQue) -> Self {
        Self { comp_que, ..self }
    }
}

// NOTE: First Bit configuration

/// PGA[11:9]: Programmable gain amplifier
/// Only for ADS1114/5
#[derive(Debug, Clone, Copy, Default)]
#[repr(u8)]
// 15 14 13 12 [Gain] 8
pub enum Gain {
    #[default]
    V6_144 = 0,
    V4_096 = 1 << 1,
    V2_048 = 2 << 1,
    V1_024 = 3 << 1,
    V0_512 = 4 << 1,
    V0_256 = 5 << 1,
}

// NOTE: Second Bit configuration

/// DR[7:5]: Data rate
#[derive(Debug, Clone, Copy, Default)]
#[repr(u8)]
// [DR] 4 3 2 1 0
pub enum DataRate {
    SPS8 = 0,
    SPS16 = 1 << 5,
    SPS32 = 2 << 5,
    SPS64 = 3 << 5,
    SPS128 = 4 << 5,
    SPS250 = 5 << 5,
    SPS475 = 6 << 5,
    #[default]
    SPS860 = 7 << 5,
}

/// COMP_MODE [4]:  Comparator mode
/// Only for ADS1114/5
#[derive(Debug, Clone, Copy, Default)]
#[repr(u8)]
// 7 6 5 [COMP_MODE] 3 2 1 0
pub enum CompMode {
    #[default]
    Traditional = 0,
    Window = 1 << 4,
}

/// COMP_POL [3]:  Comparator polarity
/// Only for ADS1114/5
#[derive(Debug, Clone, Copy, Default)]
#[repr(u8)]
// 7 6 5 4 [COMP_POL] 2 1 0
pub enum CompPol {
    #[default]
    ActiveLow = 0,
    ActiveHigh = 1 << 3,
}

/// COMP_LAT [2]: Latching Comparator
/// Only for ADS1114/5
#[derive(Debug, Clone, Copy, Default)]
#[repr(u8)]
// 7 6 5 4 3 [COMP_LAT] 1 0
pub enum CompLat {
    #[default]
    NonLatching = 0,
    Latching = 1 << 2,
}

/// COMP_QUE [1:0]: Comparator queue and disable
/// Only for ADS1114/5
#[derive(Debug, Clone, Copy, Default)]
#[repr(u8)]
// 7 6 5 4 3 2 [COMP_QUE]
pub enum CompQue {
    OneConversion = 0,
    TwoConversions = 1,
    FourConversions = 2,
    #[default]
    DisableComparator = 3,
}

#[derive(Debug, Clone, Copy, Default)]
#[repr(u8)]
pub enum Address {
    #[default]
    Ground = 0b1001000,
    VDD = 0b1001001,
    SDA = 0b1001010,
    SCL = 0b1001011,
}
