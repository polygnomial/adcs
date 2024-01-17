// ! Driver for the PNI RM3100 Magnetometer
// ! Author: Flynn Dreilinger
// ! Based on: https://os.mbed.com/users/fwrawx/code/Rm3100/
// ! 
// ! Example usage on a Teensy 4.1:
// ! // configure pull up resistors on the I2C pins
// ! iomuxc::configure(&mut pins.p18, I2C_PIN_CONFIG);
// ! iomuxc::configure(&mut pins.p19, I2C_PIN_CONFIG);
// ! // configure I2C object
// ! let mut lpi2c1: board::Lpi2c1 =
// !     board::lpi2c(lpi2c1, pins.p19, pins.p18, board::Lpi2cClockSpeed::KHz100);
// ! // Configure the RM3100 magnetometer
// ! let mut sensor = rm3100::RM3100Driver::new(lpi2c1, 0x20);
// ! sensor.begin();
// ! delay.block_ms(250);
// ! sensor.set_cycle_counts_xyz_equal(200);
// ! delay.block_ms(250);
// ! sensor.set_rate(100.0);
// ! delay.block_ms(250);
// ! sensor.set_continuous_measurement_mode_enabled(true);
// ! delay.block_ms(250);
// ! // read magnetometer
// ! let sample: rm3100::Sample = sensor.get_sample().unwrap();


use embedded_hal::blocking::i2c::{Read, Write, WriteRead};


use bytemuck;
use bitfield_struct::bitfield;

pub struct RM3100Driver<I2C> {
    i2c: I2C,
    device_addr: u8,
    scale: MeasurementScale,
    sample_delay_ms: i32,
    // other fields...
}

pub enum ReturnCode {
    RM3100_RET_EIO = -22,
    RM3100_RET_ENODEV = -19,
    RM3100_RET_EINVAL = -5,
    RM3100_RET_OK = 0,
}

pub struct I2CAddress;

impl I2CAddress {
    pub const RM3100_ADDR_SA0: u8 = 0x01; // address bit 0
    pub const RM3100_ADDR_SA1: u8 = 0x02; // address bit 1
    pub const RM3100_ADDR_MASK: u8 = 0x03;
    pub const RM3100_ADDR: u8 = 0x20;     // 7-bit base address
    pub const RM3100_ADDR_SSN: u8 = 0x01; // SSN high = address bit 0
    pub const RM3100_ADDR_SO: u8 = 0x02;  // SO high = address bit 1
}

pub struct Register;

impl Register {
    pub const RM3100_REG_POLL: u8 =   0x00; // polls for a single measurement
    pub const RM3100_REG_CMM: u8 =    0x01; // initiates continuous measurement mode
    pub const RM3100_REG_CCX: u8 =    0x04; // cycle counts -- X axis
    pub const RM3100_REG_CCY: u8 =    0x06; // cycle counts -- Y axis
    pub const RM3100_REG_CCZ: u8 =    0x08; // cycle counts -- Z axis
    pub const RM3100_REG_TMRC: u8 =   0x0B; // sets continuous mode data rate
    pub const RM3100_REG_MX: u8 =     0x24; // measurement results -- X axis
    pub const RM3100_REG_MY: u8 =     0x27; // measurement results -- Y axis
    pub const RM3100_REG_MZ: u8 =     0x2A; // measurement results -- Z axis
    pub const RM3100_REG_BIST: u8 =   0x33; // built-in self test
    pub const RM3100_REG_STATUS: u8 = 0x34; // status of DRDY
    pub const RM3100_REG_HSHAKE: u8 = 0x35; // handshake register
    pub const RM3100_REG_REVID: u8 =  0x36; // MagI2C revision identification
}

pub struct SingleMeasurementMode {
    res0: u8,
    pmx: u8,
    pmy: u8,
    pmz: u8,
    res7: u8,
}

pub enum DataReadyMode {
    RM3100_DRDM_RES0 = 0x00,
    RM3100_DRDM_ANY_AXES = 0x01,
    RM3100_DRDM_ALL_AXES = 0x02,
    RM3100_DRDM_MASK = 0x03,
}


#[bitfield(u8)]
#[derive(bytemuck::Pod, bytemuck::Zeroable)]
pub struct ContinuousMeasurementMode {
    
    start: bool,
    res1: bool,
    #[bits(2)]
    drdm: usize,
    cmx: bool,
    cmy: bool,
    cmz: bool,
    res7: bool,
}

// #[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
// #[repr(C)]
pub struct CycleCounts {
    pub x: u16,
    pub y: u16,
    pub z: u16,
}

pub struct ContinuousMeasurementModeUpdateRate;

impl ContinuousMeasurementModeUpdateRate {
    pub const RM3100_CMM_RATE_600_0_HZ: u8 = 0x02; //   ~600 Hz
    pub const RM3100_CMM_RATE_300_0_HZ: u8 = 0x03; //   ~300 Hz
    pub const RM3100_CMM_RATE_150_0_HZ: u8 = 0x04; //   ~150 Hz
    pub const RM3100_CMM_RATE_75_0_HZ: u8 = 0x05;  //    ~75 Hz
    pub const RM3100_CMM_RATE_37_0_HZ: u8 = 0x06;  //    ~37 Hz
    pub const RM3100_CMM_RATE_18_0_HZ: u8 = 0x07;  //    ~18 Hz
    pub const RM3100_CMM_RATE_9_0_HZ: u8 = 0x08;   //     ~9 Hz
    pub const RM3100_CMM_RATE_4_5_HZ: u8 = 0x09;   //   ~4.5 Hz
    pub const RM3100_CMM_RATE_2_3_HZ: u8 = 0x0A;   //   ~2.3 Hz
    pub const RM3100_CMM_RATE_1_2_HZ: u8 = 0x0B;   //   ~1.2 Hz
    pub const RM3100_CMM_RATE_0_6_HZ: u8 = 0x0C;   //   ~0.6 Hz
    pub const RM3100_CMM_RATE_0_3_HZ: u8 = 0x0D;   //   ~0.3 Hz
    pub const RM3100_CMM_RATE_0_015_HZ: u8 = 0x0E; // ~0.015 Hz
    pub const RM3100_CMM_RATE_0_075_HZ: u8 = 0x0F; // ~0.075 Hz
    pub const RM3100_CMM_RATE_MASK: u8 = 0x05;     //  ~75 Hz
    pub const RM3100_CMM_RATE_MSB: u8 = 0x90;
}

pub struct Status {
    res0: u8,
    drdy: u8,
}

pub struct Measurement {
    x: i32,
    y: i32,
    z: i32,
}

pub struct Sample {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

pub struct MeasurementScale {
    x: f32,
    y: f32,
    z: f32,
}

pub struct SelfTestCount;

impl SelfTestCount {
    pub const RM3100_SELF_TEST_COUNT_1: u8 = 0x01; // 1 LR periods
    pub const RM3100_SELF_TEST_COUNT_2: u8 = 0x02; // 2 LR periods
    pub const RM3100_SELF_TEST_COUNT_4: u8 = 0x03; // 4 LR periods
    pub const RM3100_SELF_TEST_COUNT_MASK: u8 = 0x03;
}

pub struct SelfTestTimeout;

impl SelfTestTimeout {
    pub const RM3100_SELF_TEST_TIMEOUT_30_US: u8 = 0x01;  // 1 cycle  --  30 µs
    pub const RM3100_SELF_TEST_TIMEOUT_60_US: u8 = 0x02;  // 2 cycles --  60 µs
    pub const RM3100_SELF_TEST_TIMEOUT_120_US: u8 = 0x03; // 4 cycles -- 120 µs
    pub const RM3100_SELF_TEST_TIMEOUT_MASK: u8 = 0x03;
}

pub struct SelfTestConfig {
    bp: u8,
    bw: u8,
    xok: u8,
    yok: u8,
    zok: u8,
    ste: u8,
}

pub struct HandShakeConfig {
    drc0: u8,
    drc1: u8,
    res2: u8,
    res3: u8,
    nack0: u8,
    nack1: u8,
    nack2: u8,
}

pub const RM3100_REVID: u8 = 0x22;

impl<I2C, E> RM3100Driver<I2C>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
{
    pub fn new(i2c: I2C, device_addr: u8) -> Self {
        RM3100Driver { 
            i2c, 
            device_addr,
            scale: MeasurementScale {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
            sample_delay_ms: 1,
        }
    }

    pub fn begin(&mut self) -> Result<(), ReturnCode> {
        let mut revid = [0u8, 1];
        let mut cc = CycleCounts { x: 0, y: 0, z: 0 };

        // self.i2c.begin(); // no begin needed in rust

        self.get_hardware_revision(&mut revid);
        if revid[0] != RM3100_REVID {
            return Err(ReturnCode::RM3100_RET_ENODEV);
        }

        self.get_cycle_counts(&mut cc);

        self.update_measurement_scale(&cc);

        Ok(())
    }

    pub fn get_cycle_counts(&mut self, _cc: &CycleCounts) -> Result<(), ReturnCode> {
        let mut buffer = [0u8; 6];
        self.read(Register::RM3100_REG_CCX, &mut buffer);
        let _x = u16::from_be_bytes(buffer[0..2].try_into().unwrap());
        let _y = u16::from_be_bytes(buffer[2..4].try_into().unwrap());
        let _z = u16::from_be_bytes(buffer[4..6].try_into().unwrap());
        Ok(())
    }

    pub fn set_cycle_counts(&mut self, cc: &CycleCounts) -> Result<(), E> {
        let buffer = [
            (cc.x >> 8) as u8,
            cc.x as u8,
            (cc.y >> 8) as u8,
            cc.y as u8,
            (cc.z >> 8) as u8,
            cc.z as u8,
        ];
        self.update_measurement_scale(cc);
        self.write(Register::RM3100_REG_CCX, &buffer)
    }

    pub fn set_cycle_counts_xyz(
        &mut self,
        x: u16,
        y: u16,
        z: u16,
    ) -> Result<(), E> {
        let cc = CycleCounts { x, y, z };
        self.set_cycle_counts(&cc)
    }

    pub fn set_cycle_counts_xy(&mut self, xy: u16, z: u16) -> Result<(), E> {
        self.set_cycle_counts_xyz(xy, xy, z)
    }

    pub fn set_cycle_counts_xyz_equal(&mut self, xyz: u16) -> Result<(), E> {
        self.set_cycle_counts_xy(xyz, xyz)
    }

    pub fn set_continuous_measurement_mode_update_rate(
        &mut self,
        tmrc: u8,
    ) -> Result<(), E> {
        let value = (tmrc & ContinuousMeasurementModeUpdateRate::RM3100_CMM_RATE_MASK as u8)
            | ContinuousMeasurementModeUpdateRate::RM3100_CMM_RATE_MSB as u8;
        self.write(Register::RM3100_REG_TMRC, &[value])
    }

    pub fn set_rate(&mut self, rate: f32) -> Result<(), E> {
        let mut r = 600.0;
        let mut tmrc = ContinuousMeasurementModeUpdateRate::RM3100_CMM_RATE_600_0_HZ as u8;
        while tmrc < ContinuousMeasurementModeUpdateRate::RM3100_CMM_RATE_MASK as u8
            && (r / 2.0) >= rate
        {
            r /= 2.0;
            tmrc += 1;
        }
        self.set_continuous_measurement_mode_update_rate(tmrc)
    }

    pub fn set_continuous_measurement_mode(
        &mut self,
        cmm: &ContinuousMeasurementMode,
    ) -> Result<(), E> {
        let mut _byte_slice: &[u8];
        // // bincode::serialize_into(&mut byte_slice, &cmm).unwrap();
        let byte_slice: &[u8] = bytemuck::bytes_of(cmm);
        self.write(Register::RM3100_REG_CMM, byte_slice);
        Ok(())
    }

    pub fn set_continuous_measurement_mode_xyz(
        &mut self,
        enabled: bool,
        drdm: u8,
        x: bool,
        y: bool,
        z: bool,
    ) -> Result<(), E> {
        let cmm = ContinuousMeasurementMode::new()
            .with_start(enabled)
            .with_res1(false)
            .with_drdm((drdm & DataReadyMode::RM3100_DRDM_MASK as u8) as usize)
            .with_cmx(x)
            .with_cmy(y)
            .with_cmz(z)
            .with_res7(false);
        self.set_continuous_measurement_mode(&cmm)
    }

    pub fn set_continuous_measurement_mode_enabled(
        &mut self,
        enabled: bool,
    ) -> Result<(), E> {
        self.set_continuous_measurement_mode_xyz(
            enabled,
            DataReadyMode::RM3100_DRDM_ALL_AXES as u8,
            enabled,
            enabled,
            enabled,
        )
    }

    pub fn update_measurement_scale(&mut self, cc: &CycleCounts) {
        self.scale.x = 1.0 / ((cc.x as f32 * 0.3627) + 1.85);
        self.scale.y = 1.0 / ((cc.y as f32 * 0.3627) + 1.85);
        self.scale.z = 1.0 / ((cc.z as f32 * 0.3627) + 1.85);
    }

    pub fn get_hardware_revision(&mut self, rev: &mut [u8]) -> Result<(), E> {
        // let mut rev = [0u8];
        self.read(Register::RM3100_REG_REVID, rev)?;
        Ok(())
    }

    pub fn get_measurement(&mut self) -> Result<Measurement, E> {
        let mut buffer = [0u8; 9];
        self.read(Register::RM3100_REG_MX, &mut buffer);
        let x = ((buffer[0] as i8) as i32) << 16
            | (buffer[1] as i32) << 8
            | (buffer[2] as i32);
        let y = ((buffer[3] as i8) as i32) << 16
            | (buffer[4] as i32) << 8
            | (buffer[5] as i32);
        let z = ((buffer[6] as i8) as i32) << 16
            | (buffer[7] as i32) << 8
            | (buffer[8] as i32);
        Ok(Measurement { x, y, z })
    }

    pub fn get_sample(&mut self) -> Result<Sample, E> {
        let m = self.get_measurement();
        match m {
            Ok(m) => {
                let x = m.x as f32 * self.scale.x;
                let y = m.y as f32 * self.scale.y;
                let z = m.z as f32 * self.scale.z;
                return Ok(Sample { x, y, z });
            },
            Err(_e) => {
                let x = 0.0;
                let y = 0.0;
                let z = 0.0;
                return Ok(Sample { x, y, z });
            },
        }   
    }

    pub fn read(&mut self, register_address: u8, buffer: &mut [u8]) -> Result<(), E> {
        // Write the register address to the device and then read data
        self.i2c.write_read(self.device_addr, &[register_address], buffer)?;
        Ok(())
    }

    pub fn write(&mut self, register_address: u8, data: &[u8]) -> Result<(), E> {
        // Write the register address and data to the target device
        let new_len = data.len() + 1;
        let mut data_and_addr = [0u8; 256];
        data_and_addr[0] = register_address;
        data_and_addr[1..new_len].copy_from_slice(data);
        self.i2c.write(self.device_addr, &data_and_addr[0..new_len])?;
        Ok(())
    }
}

