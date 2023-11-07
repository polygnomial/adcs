#ifndef RM3100_HPP
#define RM3100_HPP

#include <stdint.h>
#include <Wire.h>

/**
 * An interface for the RM3100 3-axis magnetometer from PNI Sensor Corp.
 *
 * @code
 * #include "mbed.h"
 * #include "Rm3100.hpp"
 * 
 * int main(void)
 * {
 *     Serial pc(USBTX, USBRX);
 *     pc.baud(115200);
 * 
 *     printf("### Hello RM3100 ###\r\n");
 * 
 *     int addr = ((Rm3100::RM3100_ADDR | Rm3100::RM3100_ADDR_SSN) << 1);
 *     struct Rm3100::Status status = { 0 };
 *     struct Rm3100::Sample sample = { 0 };
 * 
 *     Rm3100 sensor(I2C_SDA, I2C_SCL, addr);
 * 
 *     sensor.Begin();
 *     osDelay(1);
 * 
 *     sensor.SetCycleCounts(200);
 *     osDelay(1);
 * 
 *     sensor.SetRate(100.0f);
 *     osDelay(1);
 * 
 *     sensor.SetContinuousMeasurementMode(true);
 *     osDelay(1);
 * 
 *     while (true) {
 *         sensor.GetStatus(&status);
 *         if (status.drdy) {
 *             sensor.GetSample(&sample);
 *             printf("x: %f, y: %f, z: %f\r\n", sample.x, sample.y, sample.z);
 *         }
 *         osDelay(10);
 *     }
 * }
 * @endcode
 */
class Rm3100
{
public:

    enum ReturnCode {
        RM3100_RET_EIO = -22,
        RM3100_RET_ENODEV = -19,
        RM3100_RET_EINVAL = -5,
        RM3100_RET_OK = 0,
    };

    enum I2CAddress {
        RM3100_ADDR_SA0 =  0x01, // address bit 0
        RM3100_ADDR_SA1 =  0x02, // address bit 1
        RM3100_ADDR_MASK = 0x03,
        RM3100_ADDR =      0x20, // 7-bit base address
        RM3100_ADDR_SSN = RM3100_ADDR_SA0, // SSN high = address bit 0
        RM3100_ADDR_SO = RM3100_ADDR_SA1, // SO high = address bit 1
    };

    enum Register {
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
    };

    struct SingleMeasurementMode {
        uint8_t res0:4;
        uint8_t pmx:1;
        uint8_t pmy:1;
        uint8_t pmz:1;
        uint8_t res7:1;
    };

    enum DataReadyMode {
        RM3100_DRDM_RES0 =     0x00, // reserved
        RM3100_DRDM_ANY_AXES = 0x01, // drdy on measurement complete on any axis
        RM3100_DRDM_ALL_AXES = 0x02, // drdy on measurement complete on all axes
        RM3100_DRDM_MASK =     0x03,
    };

    struct ContinuousMeasurementMode {
        uint8_t start:1;   // continuous measurement mode enabled
        uint8_t res1:1;
        uint8_t drdm:2;    // data ready mode
        uint8_t cmx:1;     // X axis measurement enabled in CMM
        uint8_t cmy:1;     // Y axis measurement enabled in CMM
        uint8_t cmz:1;     // Z axis measurement enabled in CMM
        uint8_t res7:1;
    };

    struct CycleCounts {
        uint16_t x;
        uint16_t y;
        uint16_t z;
    };

    enum ContinuousMeasurementModeUpdateRate {
        RM3100_CMM_RATE_600_0_HZ = 0x02, //   ~600 Hz
        RM3100_CMM_RATE_300_0_HZ = 0x03, //   ~300 Hz
        RM3100_CMM_RATE_150_0_HZ = 0x04, //   ~150 Hz
        RM3100_CMM_RATE_75_0_HZ =  0x05, //    ~75 Hz
        RM3100_CMM_RATE_37_0_HZ =  0x06, //    ~37 Hz
        RM3100_CMM_RATE_18_0_HZ =  0x07, //    ~18 Hz
        RM3100_CMM_RATE_9_0_HZ =   0x08, //     ~9 Hz
        RM3100_CMM_RATE_4_5_HZ =   0x09, //   ~4.5 Hz
        RM3100_CMM_RATE_2_3_HZ =   0x0A, //   ~2.3 Hz
        RM3100_CMM_RATE_1_2_HZ =   0x0B, //   ~1.2 Hz
        RM3100_CMM_RATE_0_6_HZ =   0x0C, //   ~0.6 Hz
        RM3100_CMM_RATE_0_3_HZ =   0x0D, //   ~0.3 Hz
        RM3100_CMM_RATE_0_015_HZ = 0x0E, // ~0.015 Hz
        RM3100_CMM_RATE_0_075_HZ = 0x0F, // ~0.075 Hz
        RM3100_CMM_RATE_MASK = RM3100_CMM_RATE_0_075_HZ,
        RM3100_CMM_RATE_MSB =      0x90,
    };

    enum StatusFlag {
        RM3100_STATUS_FLAG_DRDY = (1 << 7),
    };

    struct Status {
        uint8_t res0:7;
        uint8_t drdy:1;
    };

    struct Measurement {
        int32_t x;
        int32_t y;
        int32_t z;
    };

    struct Sample {
        float x;
        float y;
        float z;
    };

    struct MeasurementScale {
        float x;
        float y;
        float z;
        MeasurementScale(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
    };

    enum SelfTestCount {
        RM3100_SELF_TEST_COUNT_1 = 0x01, // 1 LR periods
        RM3100_SELF_TEST_COUNT_2 = 0x02, // 2 LR periods
        RM3100_SELF_TEST_COUNT_4 = 0x03, // 4 LR periods
        RM3100_SELF_TEST_COUNT_MASK = RM3100_SELF_TEST_COUNT_4,
    };

    enum SelfTestTimeout {
        RM3100_SELF_TEST_TIMEOUT_30_US = 0x01,  // 1 cycle  --  30 µs
        RM3100_SELF_TEST_TIMEOUT_60_US = 0x02,  // 2 cycles --  60 µs
        RM3100_SELF_TEST_TIMEOUT_120_US = 0x03, // 4 cycles -- 120 µs
        RM3100_SELF_TEST_TIMEOUT_MASK = RM3100_SELF_TEST_TIMEOUT_120_US,
    };

    struct SelfTestConfig {
        uint8_t bp:2;  // test count (LR periods)
        uint8_t bw:2;  // timeout (sleep oscillation cycles)
        uint8_t xok:1; // X result -- 0 = error, 1 = ok
        uint8_t yok:1; // Y result -- 0 = error, 1 = ok
        uint8_t zok:1; // Z result -- 0 = error, 1 = ok
        uint8_t ste:1; // self test enable -- 0 = disable, 1 = enable
    };

    struct HandShakeConfig {
        uint8_t drc0:1;  // clear drdy on any register write
        uint8_t drc1:1;  // clear drdy on measurement read
        uint8_t res2:1;  // 0
        uint8_t res3:1;  // 1
        uint8_t nack0:1; // 1 when undef register write
        uint8_t nack1:1; // 1 when write SMM when CMM or visa versa
        uint8_t nack2:1; // 1 when measurement read before data ready
    };

    static const uint8_t RM3100_REVID = 0x22;

    /**
     * Creates a new instance attached to the specified I2C pins and address
     *
     * @param sda I2C data pin
     * @param scl I2C clock pin
     * @param addr 8-bit I2C address
     */
    Rm3100(TwoWire wire, uint8_t addr);

    virtual ~Rm3100();

    /**
     * Checks hardware id and sets the data scale
     *
     * @return 0 on success
     */
    int Begin(void);

    /**
     * Gets the current single measurement mode config
     *
     * @param smm pointer to read config into
     * @return 0 on success
     */
    int GetSingleMeasurementMode(struct SingleMeasurementMode *smm);

    /**
     * Sets the current single measurement mode config
     *
     * @param smm pointer to write config from
     * @return 0 on success
     */
    int SetSingleMeasurementMode(struct SingleMeasurementMode *smm);

    /**
     * Sets the current single measurement mode config
     *
     * @param x enable measurement on X axis
     * @param y enable measurement on Y axis
     * @param z enable measurement on Z axis
     * @return 0 on success
     */
    int SetSingleMeasurementMode(bool x, bool y, bool z);

    /**
     * Gets the current continuous measurement mode config
     *
     * @param cmm pointer to read config into
     * @return 0 on success
     */
    int GetContinuousMeasurementMode(struct ContinuousMeasurementMode *cmm);

    /**
     * Sets the current continuous measurement mode config
     *
     * @param cmm pointer to write config from
     * @return 0 on success
     */
    int SetContinuousMeasurementMode(struct ContinuousMeasurementMode *cmm);

    /**
     * Sets the current continuous measurement mode config
     *
     * @param enabled enable CMM -- true = enabled, false = disabled
     * @param drdm data ready mode
     * @param x enable measurement on X axis
     * @param y enable measurement on Y axis
     * @param z enable measurement on Z axis
     * @return 0 on success
     */
    int SetContinuousMeasurementMode(bool enabled, uint8_t drdm, bool x,
                                     bool y, bool z);

    /**
     * Sets the current continuous measurement mode config
     *
     * @param enabled enable CMM -- true = enabled on all axes, false = disabled
     * @return 0 on success
     */
    int SetContinuousMeasurementMode(bool enabled);

    /**
     * Gets the current cycle counts
     *
     * @param cc pointer to read into
     * @return 0 on success
     */
    int GetCycleCounts(struct CycleCounts *cc);

    /**
     * Sets the current cycle counts
     *
     * @param cc pointer to write from
     * @return 0 on success
     */
    int SetCycleCounts(struct CycleCounts *cc);

    /*
     * Sets the current cycle counts (x, y, z)
     *
     * @param x cycle counts for X axis
     * @param y cycle counts for Y axis
     * @param z cycle counts for Z axis
     * @return 0 on success
     */
    int SetCycleCounts(uint16_t x, uint16_t y, uint16_t z);

    /*
     * Sets the current cycle counts (x = y, z)
     *
     * @param xy cycle counts for X and Y axes
     * @param z cycle counts for Z axis
     * @return 0 on success
     */
    int SetCycleCounts(uint16_t xy, uint16_t z);

    /*
     * Sets the current cycle counts (x = y = z)
     *
     * @param xyz cycle counts for all axes
     * @return 0 on success
     */
    int SetCycleCounts(uint16_t xyz);

    /**
     * Updates the measurement scale based on the given cycle counts
     *
     * @param cc pointer to cycle counts
     * @return 0 on success
     */
    void UpdateMeasurementScale(struct CycleCounts *cc);

    /**
     * Gets the continuous mode update rate
     *
     * @param tmrc pointer to read into
     * @return 0 on success
     */
    int GetContinuousMeasurementModeUpdateRate(uint8_t *tmrc);

    /**
     * Sets the continuous mode update rate (TMRC)
     *
     * @param tmrc rate to set according to TMRC table
     * @return 0 on success
     */
    int SetContinuousMeasurementModeUpdateRate(uint8_t tmrc);

    /**
     * Sets the contiunous mode update rate (Hz)
     *
     * @param rate rate to set in Hz
     * @return 0 on success
     */
    int SetRate(float rate);

    /**
     * Gets the current status
     *
     * @param status pointer to read into
     * @return 0 on success
     */
    int GetStatus(struct Status *status);

    /**
     * Gets the current measurement data (24-bit signed)
     *
     * @param m pointer to read into
     * @return 0 on success
     */
    int GetMeasurement(struct Measurement *m);

    /**
     * Gets the current measurement sample (scale to µT)
     *
     * @param s pointer to read into
     * @return 0 on success
     */
    int GetSample(struct Sample *s);

    /**
     * Gets the current self-test config/result
     *
     * @param cfg pointer to read into
     * @return 0 on success
     */
    int GetSelfTestConfig(struct SelfTestConfig *cfg);

    /**
     * Sets the current self-test config
     *
     * @param cfg pointer to write from
     * @return 0 on success
     */
    int SetSelfTestConfig(struct SelfTestConfig *cfg);

    /**
     * Sets the current self-test config
     *
     * @param count LR periods
     * @param timeout sleep oscillation cycles
     * @return 0 on success
     */
    int SetSelfTestConfig(uint8_t count, uint8_t timeout, bool enabled);

    /**
     * Performs a self-test, returning result
     *
     * @param count LR periods
     * @param timeout sleep oscillation cycles
     * @param result pointer to read result into
     * @return 0 on success
     */
    int PerformSelfTest(uint8_t count, uint8_t timeout,
                        struct SelfTestConfig *result);

    /**
     * Clears the self-test config
     *
     * @return 0 on success
     */
    int ClearSelfTest();

    /**
     * Gets the current handshake config
     *
     * @param cfg pointer to read into
     * @return 0 on success
     */
    int GetHandShakeConfig(struct HandShakeConfig *cfg);

    /**
     * Sets the current handshake config
     *
     * @param cfg pointer to write from
     * @return 0 on success
     */
    int SetHandShakeConfig(struct HandShakeConfig *cfg);

    /**
     * Sets the current data ready config
     *
     * @param on_write 1 = drdy cleared on any register write
     * @param on_read_measurement 1 = drdy cleared on measurement register read
     * @return 0 on success
     */
    int SetDrdyClearConfig(bool on_write, bool on_read_measurement);

    /**
     * Gets the current hardware revision
     *
     * @param rev pointer to read into
     * @return 0 on success
     */
    int GetHardwareRevision(uint8_t *rev);

private:
    TwoWire _wire;
    uint8_t _addr;
    struct MeasurementScale _scale;
    int _sample_delay_ms;
    int Read(uint8_t reg, uint8_t *buffer, uint8_t count);
    int Write(uint8_t reg, uint8_t *buffer, uint8_t count);
};

#endif /* RM3100_HPP */