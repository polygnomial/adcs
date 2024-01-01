
use embedded_hal::blocking::i2c::{Write, WriteRead};
use embedded_hal::blocking::delay::DelayMs;

pub struct RM3100Driver<I2C> {
    i2c: I2C,
    addr: u8,
    scale: MeasurementScale,
    sample_delay_ms: i32,
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
    pub const RM3100_ADDR: u8 = 0x20; // 7-bit base address
    pub const RM3100_ADDR_SSN: u8 = 0x01; // SSN high = address bit 0
    pub const RM3100_ADDR_SO: u8 = 0x02;  // SO high = address bit 1
}

pub enum Register {
    RM3100_REG_POLL =   0x00, // polls for a single measurement
    RM3100_REG_CMM =    0x01, // initiates continuous measurement mode
    RM3100_REG_CCX =    0x04, // cycle counts -- X axis
    RM3100_REG_CCY =    0x06, // cycle counts -- Y axis
    RM3100_REG_CCZ =    0x08, // cycle counts -- Z axis
    RM3100_REG_TMRC =   0x0B, // sets continuous mode data rate
    RM3100_REG_MX =     0x24, // measurement results -- X axis
    RM3100_REG_MY =     0x27, // measurement results -- Y axis
    RM3100_REG_MZ =     0x2A, // measurement results -- Z axis
    RM3100_REG_BIST =   0x33, // built-in self test
    RM3100_REG_STATUS = 0x34, // status of DRDY
    RM3100_REG_HSHAKE = 0x35, // handshake register
    RM3100_REG_REVID =  0x36, // MagI2C revision identification
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

pub struct ContinuousMeasurementMode {
    start: u8,
    res1: u8,
    drdm: u8,
    cmx: u8,
    cmy: u8,
    cmz: u8,
    res7: u8,
}

pub struct CycleCounts {
    x: u16,
    y: u16,
    z: u16,
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
    x: f32,
    y: f32,
    z: f32,
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
    I2C: WriteRead<Error = E>,
{
    pub fn new(i2c: I2C, addr: u8) -> Self {
        RM3100Driver {
            i2c,
            addr,
            scale: MeasurementScale {
                x: 1.0,
                y: 1.0,
                z: 1.0,
            },
            sample_delay_ms: 1,
        }
    }

    pub fn begin(&mut self) -> Result<(), ReturnCode> {
        let mut revid: u8 = 0x00;
        let mut cc = CycleCounts { x: 0, y: 0, z: 0 };

        // self.i2c.begin(); // no begin needed in rust

        self.get_hardware_revision(&mut revid)?;

        if revid != RM3100_REVID {
            return Err(ReturnCode::RM3100_RET_ENODEV);
        }

        self.get_cycle_counts(&mut cc)?;

        self.update_measurement_scale(&cc);

        Ok(())
    }

    pub fn get_single_measurement_mode(&mut self) -> Result<SingleMeasurementMode, ReturnCode> {
        let mut smm = SingleMeasurementMode {
            res0: 0,
            pmx: 0,
            pmy: 0,
            pmz: 0,
            res7: 0,
        };
        self.read(Register::RM3100_REG_POLL, &mut smm)?;
        Ok(smm)
    }

    pub fn set_single_measurement_mode(
        &mut self,
        smm: &SingleMeasurementMode,
    ) -> Result<(), ReturnCode> {
        self.write(Register::RM3100_REG_POLL, smm)
    }

    pub fn set_single_measurement_mode_xyz(
        &mut self,
        x: bool,
        y: bool,
        z: bool,
    ) -> Result<(), ReturnCode> {
        let smm = SingleMeasurementMode {
            res0: 0,
            pmx: x as u8,
            pmy: y as u8,
            pmz: z as u8,
            res7: 0,
        };
        self.set_single_measurement_mode(&smm)
    }

    pub fn get_continuous_measurement_mode(
        &mut self,
    ) -> Result<ContinuousMeasurementMode, ReturnCode> {
        let mut cmm = ContinuousMeasurementMode {
            start: 0,
            res1: 0,
            drdm: 0,
            cmx: 0,
            cmy: 0,
            cmz: 0,
            res7: 0,
        };
        self.read(Register::RM3100_REG_CMM, &mut cmm)?;
        Ok(cmm)
    }

    pub fn set_continuous_measurement_mode(
        &mut self,
        cmm: &ContinuousMeasurementMode,
    ) -> Result<(), ReturnCode> {
        self.write(Register::RM3100_REG_CMM, cmm)
    }

    pub fn set_continuous_measurement_mode_xyz(
        &mut self,
        enabled: bool,
        drdm: u8,
        x: bool,
        y: bool,
        z: bool,
    ) -> Result<(), ReturnCode> {
        let cmm = ContinuousMeasurementMode {
            start: enabled as u8,
            res1: 0,
            drdm: drdm & DataReadyMode::RM3100_DRDM_MASK as u8,
            cmx: x as u8,
            cmy: y as u8,
            cmz: z as u8,
            res7: 0,
        };
        self.set_continuous_measurement_mode(&cmm)
    }

    pub fn set_continuous_measurement_mode_enabled(
        &mut self,
        enabled: bool,
    ) -> Result<(), ReturnCode> {
        self.set_continuous_measurement_mode_xyz(
            enabled,
            DataReadyMode::RM3100_DRDM_ALL_AXES as u8,
            enabled,
            enabled,
            enabled,
        )
    }

    pub fn get_cycle_counts(&mut self, cc: &CycleCounts) -> Result<CycleCounts, ReturnCode> {
        let mut buffer = [0u8; 6];
        self.read(Register::RM3100_REG_CCX, &mut buffer)?;
        let x = u16::from_be_bytes(buffer[0..2].try_into().unwrap());
        let y = u16::from_be_bytes(buffer[2..4].try_into().unwrap());
        let z = u16::from_be_bytes(buffer[4..6].try_into().unwrap());
        Ok(CycleCounts { x, y, z })
    }

    pub fn set_cycle_counts(&mut self, cc: &CycleCounts) -> Result<(), ReturnCode> {
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
    ) -> Result<(), ReturnCode> {
        let cc = CycleCounts { x, y, z };
        self.set_cycle_counts(&cc)
    }

    pub fn set_cycle_counts_xy(&mut self, xy: u16, z: u16) -> Result<(), ReturnCode> {
        self.set_cycle_counts_xyz(xy, xy, z)
    }

    pub fn set_cycle_counts_xyz_equal(&mut self, xyz: u16) -> Result<(), ReturnCode> {
        self.set_cycle_counts_xy(xyz, xyz)
    }

    pub fn update_measurement_scale(&mut self, cc: &CycleCounts) {
        self.scale.x = 1.0 / ((cc.x as f32 * 0.3627) + 1.85);
        self.scale.y = 1.0 / ((cc.y as f32 * 0.3627) + 1.85);
        self.scale.z = 1.0 / ((cc.z as f32 * 0.3627) + 1.85);
    }

    pub fn get_continuous_measurement_mode_update_rate(
        &mut self,
    ) -> Result<u8, ReturnCode> {
        let mut tmrc = 0u8;
        self.read(Register::RM3100_REG_TMRC, &mut tmrc)?;
        Ok(tmrc)
    }

    pub fn set_continuous_measurement_mode_update_rate(
        &mut self,
        tmrc: u8,
    ) -> Result<(), ReturnCode> {
        let value = (tmrc & ContinuousMeasurementModeUpdateRate::RM3100_CMM_RATE_MASK as u8)
            | ContinuousMeasurementModeUpdateRate::RM3100_CMM_RATE_MSB as u8;
        self.write(Register::RM3100_REG_TMRC, &[value])
    }

    pub fn set_rate(&mut self, rate: f32) -> Result<(), ReturnCode> {
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

    pub fn get_status(&mut self) -> Result<Status, ReturnCode> {
        let mut status = Status { res0: 0, drdy: 0 };
        self.read(Register::RM3100_REG_STATUS, &mut status)?;
        Ok(status)
    }

    pub fn get_measurement(&mut self, m: &Measurement) -> Result<Measurement, ReturnCode> {
        let mut buffer = [0u8; 9];
        self.read(Register::RM3100_REG_MX, &mut buffer)?;
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

    pub fn get_sample(&mut self) -> Result<Sample, ReturnCode> {
        let mut m = Measurement { x: 0, y: 0, z: 0 };
        self.get_measurement(&mut m, )?;
        let x = m.x as f32 * self.scale.x;
        let y = m.y as f32 * self.scale.y;
        let z = m.z as f32 * self.scale.z;
        Ok(Sample { x, y, z })
    }

    pub fn get_self_test_config(&mut self, cfg: &mut SelfTestConfig) -> Result<SelfTestConfig, ReturnCode> {
        self.read(Register::RM3100_REG_BIST, &mut cfg)?;
        Ok(*cfg)
    }

    pub fn set_self_test_config(
        &mut self,
        cfg: &SelfTestConfig,
    ) -> Result<(), ReturnCode> {
        self.write(Register::RM3100_REG_BIST, cfg)
    }

    pub fn set_self_test_config_count_timeout_enabled(
        &mut self,
        count: u8,
        timeout: u8,
        enabled: bool,
    ) -> Result<(), ReturnCode> {
        let cfg = SelfTestConfig {
            bp: count & SelfTestCount::RM3100_SELF_TEST_COUNT_MASK as u8,
            bw: timeout & SelfTestTimeout::RM3100_SELF_TEST_TIMEOUT_MASK as u8,
            xok: 0,
            yok: 0,
            zok: 0,
            ste: enabled as u8,
        };
        self.set_self_test_config(&cfg)
    }

    pub fn perform_self_test(
        &mut self,
        count: u8,
        timeout: u8,
        result: &mut SelfTestConfig,
    ) -> Result<(), ReturnCode> {
        // TODO fix and test this function
        self.set_self_test_config_count_timeout_enabled(count, timeout, true)?;
        // delay.block_ms(250);
        self.set_single_measurement_mode_xyz(true, true, true)?;
        // delay.block_ms(250);
        self.get_self_test_config(result)?;
        // delay.block_ms(250);
        self.clear_self_test()
    }

    pub fn clear_self_test(&mut self) -> Result<(), ReturnCode> {
        let value = 0u8;
        self.write(Register::RM3100_REG_BIST, &[value])
    }

    pub fn get_handshake_config(&mut self) -> Result<HandShakeConfig, ReturnCode> {
        let mut cfg = HandShakeConfig {
            drc0: 0,
            drc1: 0,
            res2: 0,
            res3: 0,
            nack0: 0,
            nack1: 0,
            nack2: 0,
        };
        self.read(Register::RM3100_REG_HSHAKE, &mut cfg)?;
        Ok(cfg)
    }

    pub fn set_handshake_config(
        &mut self,
        cfg: &HandShakeConfig,
    ) -> Result<(), ReturnCode> {
        let mut cfg = cfg.clone();
        cfg.res2 = 0;
        cfg.res3 = 1;
        self.write(Register::RM3100_REG_HSHAKE, &cfg)
    }

    pub fn set_drdy_clear_config(
        &mut self,
        on_write: bool,
        on_read_measurement: bool,
    ) -> Result<(), ReturnCode> {
        let cfg = HandShakeConfig {
            drc0: on_write as u8,
            drc1: on_read_measurement as u8,
            res2: 0,
            res3: 0,
            nack0: 0,
            nack1: 0,
            nack2: 0,
        };
        self.set_handshake_config(&cfg)
    }

    pub fn get_hardware_revision(&mut self, rev: &u8) -> Result<u8, ReturnCode> {
        let mut rev = [0u8];
        self.read(Register::RM3100_REG_REVID, &mut rev)?;
        Ok(rev)
    }

    fn read<T>(&mut self, reg: Register, buffer: &mut [u8]) -> Result<(), ReturnCode> {
        let _reg = [reg as u8];
        self.i2c.write_read(self.addr, &_reg, buffer);
        if Ok(buffer[0]) > 0 {
            Ok(())
        } else {
            Err(ReturnCode::RM3100_RET_EIO)
        }
    }

    fn write<T>(&mut self, reg: Register, buffer: &T) -> Result<(), ReturnCode> {
        // let data = [reg as u8].iter().chain(buffer.iter()).copied().collect::<Vec<u8>>();
        let result = self.i2c.write(self.addr, &buffer);
        if Ok(result) > 0 {
            Ok(())
        } else {
            Err(ReturnCode::RM3100_RET_EIO)
        }
    }
}