
use libm::floorf;

#[derive(Debug, Clone)]
pub struct Time {
    pub hour: u8,
    pub minute: u8,
    pub second: u8,
    pub millisecond: u8,
}

#[derive(Debug, Clone)]
pub struct Date {
    pub day: u8,
    pub month: u8,
    pub year: u16,
}

pub fn utc_to_jd(date: Date, time: Time) -> Result<f32, ()> {
    let mut year = date.year as f32;
    let mut month = date.month as f32;
    let day = date.day as f32;
    let hour = time.hour as f32;
    let minute = time.minute as f32;
    let second = time.second as f32;
    let millisecond = time.millisecond as f32;

    if month <= 2.0{
        year = year - 1.0;
        month = month + 12.0;
    }

    let B: f32 = floorf(year/400.0) - floorf(year/100.0) + floorf(year/4.0);
    let mjd: f32 = 365.0*year - 679004.0 + B + floorf(30.6001*(month+1.0)) + day
        + (((((millisecond / 1000.0 + second) / 60.0 + minute) / 60.0 + hour) / 24.0));
    let jd: f32 = mjd + 2400000.5;

    Ok(jd)
}