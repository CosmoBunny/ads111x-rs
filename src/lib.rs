use embedded_hal::i2c::I2c;

pub struct ADS111x<I2C: I2c, const MODEL: u8> {
    address: Address,
    i2c: I2C,
    pub config: Config,
}

pub type ADS1115<I2C> = ADS111x<I2C, 5>;
pub type ADS1114<I2C> = ADS111x<I2C, 4>;
pub type ADS1113<I2C> = ADS111x<I2C, 3>;

impl<I2C: I2c> ADS1113<I2C> {
    pub fn new(i2c: I2C, config: Config) -> Self {
        Self {
            address: Address::Ground,
            i2c,
            config,
        }
    }
    fn read_channel_raw(&mut self) -> Result<i16, I2C::Error> {
        let mut config = [1 << 7, (self.config.data_rate as u8)];
        let mut data = [0u8; 2];

        self.i2c.write_read(
            self.address as u8,
            &[0x01, config[0], config[1]],
            &mut config,
        )?;

        while ((config[0] >> 7) & 0b1) == 1 {
            self.i2c
                .write_read(self.address as u8, &[0x01], &mut config)?;
        }

        self.i2c
            .write_read(self.address as u8, &[0x00], &mut data)?;
        Ok(i16::from_be_bytes(data))
    }

    pub fn read_adc(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_raw()
    }
}

impl<I2C: I2c> ADS1114<I2C> {
    pub fn new(i2c: I2C, config: Config) -> Self {
        Self {
            address: Address::Ground,
            i2c,
            config,
        }
    }
    fn read_channel_raw(&mut self) -> Result<i16, I2C::Error> {
        let mut config = [
            (self.config.gain as u8) | (1 << 7),
            (self.config.comp_que as u8)
                | (self.config.comp_lat as u8)
                | (self.config.comp_pol as u8)
                | (self.config.comp_mode as u8)
                | (self.config.data_rate as u8),
        ];
        let mut data = [0u8; 2];

        self.i2c.write_read(
            self.address as u8,
            &[0x01, config[0], config[1]],
            &mut config,
        )?;

        while ((config[0] >> 7) & 0b1) == 1 {
            self.i2c
                .write_read(self.address as u8, &[0x01], &mut config)?;
        }

        self.i2c
            .write_read(self.address as u8, &[0x00], &mut data)?;
        Ok(i16::from_be_bytes(data))
    }

    pub fn read_adc(&mut self) -> Result<i16, I2C::Error> {
        self.read_channel_raw()
    }
}

impl<I2C: I2c> ADS1115<I2C> {
    pub fn new(address: Address, i2c: I2C, config: Config) -> Self {
        Self {
            address,
            i2c,
            config,
        }
    }
    fn read_channel_raw(&mut self, mux: u8) -> Result<i16, I2C::Error> {
        let mut config = [
            (self.config.gain as u8) | (1 << 7) | (mux << 4),
            (self.config.comp_que as u8)
                | (self.config.comp_lat as u8)
                | (self.config.comp_pol as u8)
                | (self.config.comp_mode as u8)
                | (self.config.data_rate as u8),
        ];
        let mut data = [0u8; 2];

        self.i2c.write_read(
            self.address as u8,
            &[0x01, config[0], config[1]],
            &mut config,
        )?;

        while ((config[0] >> 7) & 0b1) == 1 {
            self.i2c
                .write_read(self.address as u8, &[0x01], &mut config)?;
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
