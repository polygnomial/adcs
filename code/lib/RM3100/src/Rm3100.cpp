#include <stdint.h>
#include "Rm3100.hpp"
#include <Wire.h>
#include "Arduino.h"

Rm3100::Rm3100(TwoWire wire, uint8_t addr)
    : _wire(wire), _addr(addr), _scale(1.0f, 1.0f, 1.0f),
      _sample_delay_ms(1) {}

Rm3100::~Rm3100() {}

int Rm3100::Begin(void)
{
    uint8_t revid = 0x00;
    struct CycleCounts cc = { 0 };
    int ret;
    _wire.begin();

    // get hardware revision and validate
    ret = GetHardwareRevision(&revid);
    if (ret) {
        return ret;
    }

    if (revid != RM3100_REVID) {
        return RM3100_RET_ENODEV;
    }

    // get current cycle counts
    ret = GetCycleCounts(&cc);
    if (ret) {
        return ret;
    }

    // update scale
    UpdateMeasurementScale(&cc);

    return RM3100_RET_OK;
}

/*
 * (0x00) POLL
 */
int Rm3100::GetSingleMeasurementMode(struct SingleMeasurementMode *smm)
{
    return Read(RM3100_REG_POLL, (uint8_t *)smm, sizeof(*smm));
}

int Rm3100::SetSingleMeasurementMode(struct SingleMeasurementMode *smm)
{
    return Write(RM3100_REG_POLL, (uint8_t *)smm, sizeof(*smm));
}

int Rm3100::SetSingleMeasurementMode(bool x, bool y, bool z)
{
    struct SingleMeasurementMode smm = {
        .res0 = 0,
        .pmx = x,
        .pmy = y,
        .pmz = z,
        .res7 = 0,
    };
    return SetSingleMeasurementMode(&smm);
}

/*
 * (0x01) CMM
 */
int Rm3100::GetContinuousMeasurementMode(struct ContinuousMeasurementMode *cmm)
{
    return Read(RM3100_REG_CMM, (uint8_t *)cmm, sizeof(*cmm));
}

int Rm3100::SetContinuousMeasurementMode(struct ContinuousMeasurementMode *cmm)
{
    return Write(RM3100_REG_CMM, (uint8_t *)cmm, sizeof(*cmm));
}

int Rm3100::SetContinuousMeasurementMode(bool enabled, uint8_t drdm, bool x,
        bool y, bool z)
{
    struct ContinuousMeasurementMode cmm = {
        .start = enabled,
        .res1 = 0,
        .drdm = (drdm & RM3100_DRDM_MASK),
        .cmx = x,
        .cmy = y,
        .cmz = z,
        .res7 = 0,
    };
    return SetContinuousMeasurementMode(&cmm);
}

int Rm3100::SetContinuousMeasurementMode(bool enabled)
{
    return SetContinuousMeasurementMode(enabled, RM3100_DRDM_ALL_AXES, enabled,
                                        enabled, enabled);
}

/*
 * (0x04 -- 0x09) CCX, CCY, CCZ
 */
int Rm3100::GetCycleCounts(struct CycleCounts *cc)
{
    uint8_t buffer[6];
    int ret = Read(RM3100_REG_CCX, buffer, sizeof(buffer));
    if (ret) {
        return ret;
    }
    cc->x = (buffer[0] << 8) | buffer[1];
    cc->y = (buffer[2] << 8) | buffer[3];
    cc->z = (buffer[4] << 8) | buffer[5];
    return RM3100_RET_OK;
}

int Rm3100::SetCycleCounts(struct CycleCounts *cc)
{
    // convert to BE
    uint8_t buffer[] = {
        (cc->x >> 8), (cc->x & 0xFF),
        (cc->y >> 8), (cc->y & 0xFF),
        (cc->z >> 8), (cc->z & 0xFF),
    };
    UpdateMeasurementScale(cc);
    return Write(RM3100_REG_CCX, buffer, sizeof(buffer));
}

int Rm3100::SetCycleCounts(uint16_t x, uint16_t y, uint16_t z)
{
    struct CycleCounts cc = {
        .x = x,
        .y = y,
        .z = z,
    };
    return SetCycleCounts(&cc);
}

int Rm3100::SetCycleCounts(uint16_t xy, uint16_t z)
{
    return SetCycleCounts(xy, xy, z);
}

int Rm3100::SetCycleCounts(uint16_t xyz)
{
    return SetCycleCounts(xyz, xyz, xyz);
}

void Rm3100::UpdateMeasurementScale(struct CycleCounts *cc)
{
    _scale.x = 1.0f / ((cc->x * 0.3627f) + 1.85f);
    _scale.y = 1.0f / ((cc->y * 0.3627f) + 1.85f);
    _scale.z = 1.0f / ((cc->z * 0.3627f) + 1.85f);
}

/*
 * (0x0B) TMRC
 */
int Rm3100::GetContinuousMeasurementModeUpdateRate(uint8_t *tmrc)
{
    return Read(RM3100_REG_TMRC, tmrc, sizeof(*tmrc));
}

int Rm3100::SetContinuousMeasurementModeUpdateRate(uint8_t tmrc)
{
    uint8_t value = (tmrc & RM3100_CMM_RATE_MASK) | RM3100_CMM_RATE_MSB;
    return Write(RM3100_REG_TMRC, &value, sizeof(value));
}

int Rm3100::SetRate(float rate)
{
    float r = 600.0f;
    uint8_t tmrc = RM3100_CMM_RATE_600_0_HZ;

    while ((tmrc < RM3100_CMM_RATE_MASK) && ((r / 2.0f) >= rate)) {
        r /= 2.0f;
        tmrc++;
    }
    return SetContinuousMeasurementModeUpdateRate(tmrc);
}

int Rm3100::GetStatus(struct Status *status)
{
    return Read(RM3100_REG_STATUS, (uint8_t *)status, sizeof(*status));
}

int Rm3100::GetMeasurement(struct Measurement *m)
{
    uint8_t buffer[9] = { 0 };
    int ret = Read(RM3100_REG_MX, buffer, sizeof(buffer));
    if (ret) {
        return ret;
    }
    m->x = (((int8_t)buffer[0]) << 16) | (buffer[1] << 8) | (buffer[2]);
    m->y = (((int8_t)buffer[3]) << 16) | (buffer[4] << 8) | (buffer[5]);
    m->z = (((int8_t)buffer[6]) << 16) | (buffer[7] << 8) | (buffer[8]);
   
    return RM3100_RET_OK;
}

int Rm3100::GetSample(struct Sample *s)
{
    struct Measurement m = { 0 };
    int ret = GetMeasurement(&m);
    if (ret) {
        return ret;
    }
    s->x = (float)m.x * _scale.x;
    s->y = (float)m.y * _scale.y;
    s->z = (float)m.z * _scale.z;
    return RM3100_RET_OK;
}

int Rm3100::GetSelfTestConfig(struct SelfTestConfig *cfg)
{
    return Read(RM3100_REG_BIST, (uint8_t *)cfg, sizeof(*cfg));
}

int Rm3100::SetSelfTestConfig(struct SelfTestConfig *cfg)
{
    return Write(RM3100_REG_BIST, (uint8_t *)cfg, sizeof(*cfg));
}

int Rm3100::SetSelfTestConfig(uint8_t count, uint8_t timeout, bool enabled)
{
    struct SelfTestConfig cfg;
    cfg.bp = count & RM3100_SELF_TEST_COUNT_MASK;
    cfg.bw = timeout & RM3100_SELF_TEST_TIMEOUT_MASK;
    cfg.ste = enabled;
    return SetSelfTestConfig(&cfg);
}

int Rm3100::PerformSelfTest(uint8_t count, uint8_t timeout,
                            struct SelfTestConfig *result)
{
    // configure test
    int ret = SetSelfTestConfig(count, timeout, true);
    if (ret) {
        return ret;
    }

    delay(250);

    // initiate single measurement
    ret = SetSingleMeasurementMode(true, true, true);
    if (ret) {
        return ret;
    }

    // wait 1 ms for measurement
    delay(250);

    // get result
    ret = GetSelfTestConfig(result);
    if (ret) {
        return ret;
    }

    delay(250);

    return ClearSelfTest();
}

int Rm3100::ClearSelfTest()
{
    uint8_t value = 0;
    return Write(RM3100_REG_BIST, &value, sizeof(value));
}

int Rm3100::GetHandShakeConfig(struct HandShakeConfig *cfg)
{
    return Read(RM3100_REG_HSHAKE, (uint8_t *)cfg, sizeof(*cfg));
}

int Rm3100::SetHandShakeConfig(struct HandShakeConfig *cfg)
{
    cfg->res2 = 0;
    cfg->res3 = 1;
    return Write(RM3100_REG_HSHAKE, (uint8_t *)cfg, sizeof(*cfg));
}

int Rm3100::SetDrdyClearConfig(bool on_write, bool on_read_measurement)
{
    struct HandShakeConfig cfg = {
        .drc0 = on_write,
        .drc1 = on_read_measurement,
    };
    return SetHandShakeConfig(&cfg);
}

int Rm3100::GetHardwareRevision(uint8_t *rev)
{
    return Read(RM3100_REG_REVID, rev, sizeof(*rev));
}

int Rm3100::Read(uint8_t reg, uint8_t *buffer, uint8_t count)
{   
    // address the device
    _wire.beginTransmission(_addr);

    // write the register address
    _wire.write(reg);

    // end transmission
    _wire.endTransmission();

    // request data from the device
    uint8_t bytes_recvd = _wire.requestFrom(_addr, count);
    
    // read the data
    uint8_t counter = 0;
    while (_wire.available()){
        char byte = _wire.read();
        buffer[counter] = byte;
        counter++;
    }
    
    return counter ? RM3100_RET_OK : RM3100_RET_EIO;
}

int Rm3100::Write(uint8_t reg, uint8_t *buffer, uint8_t count)
{
    // first byte is register address
    uint8_t data[count + 1];
    data[0] = reg;
    memcpy(&data[1], buffer, count);

    // transmit to device at address
    _wire.beginTransmission(_addr);
    _wire.write((const char *)data, count + 1);
    uint8_t bytes_sent = _wire.endTransmission();

    return bytes_sent ? RM3100_RET_OK : RM3100_RET_EIO;
}
