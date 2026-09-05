#![no_std]
//! ADS1113/ADS1114/ADS1115 driver.
//!
//! ## Audit summary (pre-existing flaws fixed by this redesign)
//! - Async impl busy-polled CONFIG (`write_read` in a tight loop) — wastes bus/CPU.
//! - `enable_conversion_ready_pin` existed but was disconnected from async flow.
//! - Sync/async config-byte construction duplicated; raw `u8` MUX allowed invalid values.
//! - `Config` comparator/PGA fields applied to ADS1113 which lacks them.
//! - No mode (single-shot/continuous), polarity, timeout, or stale-data protection.
//!
//! ## Architecture
//! ```text
//! START CONVERSION -> await ALERT/RDY event (preferred) or timed delay -> READ CONVERSION
//! ```
//! Core crate depends only on `embedded-hal(-async)` plus two small generic traits
//! (`ConversionReady`, `AsyncDelayUs`) that Embassy types can implement downstream.
//! No `embassy_*` dependency. No alloc.
//!
//! ## Sampling semantics (control-loop guidance)
//! Continuous mode is **latest-sample acquisition**, not guaranteed-sample
//! acquisition: each ADS111x has one conversion register and no FIFO, so a
//! slow reader loses intermediate samples. Multi-device capture overlaps
//! conversion windows but does NOT phase-lock sampling. "Differential" means a
//! differential MUX pair on the single converter — never simultaneous sampling.

#[cfg(feature = "async")]
#[allow(unused_imports)]
use embedded_hal_async::i2c::I2c as AsyncI2c;
use embedded_hal::i2c::I2c;

/// Register pointers.
const REG_CONVERSION: u8 = 0x00;
const REG_CONFIG: u8 = 0x01;
const REG_LO_THRESH: u8 = 0x02;
const REG_HI_THRESH: u8 = 0x03;

/// OS bit: write 1 to start single-shot conversion; reads 1 when complete/idle.
const OS_MASK: u8 = 0x80;
/// Returns true if OS bit indicates conversion complete.
#[inline]
pub fn conversion_complete(cfg_hi: u8) -> bool {
    cfg_hi & OS_MASK != 0
}

pub struct ADS111x<I2C, const MODEL: u8> {
    address: Address,
    i2c: I2C,
    pub config: Config,
}

pub type ADS1115<I2C> = ADS111x<I2C, 5>;
pub type ADS1114<I2C> = ADS111x<I2C, 4>;
pub type ADS1113<I2C> = ADS111x<I2C, 3>;

/// Unified error: preserves I2C error, adds async wait failures. No alloc.
///
/// Note: there is deliberately NO timeout variant. Timeout/cancellation policy
/// belongs to the application layer (e.g. Embassy's `with_timeout` around the
/// RDY wait); a `no_std` HAL driver cannot choose a sensible deadline itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<I2cError, WaitError = ()> {
    I2c(I2cError),
    Wait(WaitError),
}

impl<I2cError, WaitError> From<I2cError> for Error<I2cError, WaitError> {
    fn from(e: I2cError) -> Self {
        Self::I2c(e)
    }
}

/// Input MUX selection (typed; replaces raw `u8`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Mux {
    A0N1 = 0b000,
    A0N3 = 0b001,
    A1N3 = 0b010,
    A2N3 = 0b011,
    A0 = 0b100,
    A1 = 0b101,
    A2 = 0b110,
    A3 = 0b111,
}

/// Operating mode.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Mode {
    #[default]
    SingleShot,
    Continuous,
}

/// Expected ALERT/RDY edge; must match `Config.comp_pol`.
/// ActiveLow -> wait for falling edge; ActiveHigh -> rising edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConversionReadyPolarity {
    ActiveLow,
    ActiveHigh,
}

impl From<CompPol> for ConversionReadyPolarity {
    fn from(p: CompPol) -> Self {
        match p {
            CompPol::ActiveLow => Self::ActiveLow,
            CompPol::ActiveHigh => Self::ActiveHigh,
        }
    }
}

/// Generic conversion-ready signal (Embassy GPIO input implements this downstream).
/// Implementations MUST be interrupt/event driven (`wait_for_falling_edge`-style),
/// never `while pin.is_high() { Timer::after().await }`.
pub trait ConversionReady {
    type Error;
    type WaitFuture<'a>: core::future::Future<Output = Result<(), Self::Error>> + 'a
    where
        Self: 'a;
    /// Synchronously arm the edge detector for `polarity`.
    ///
    /// # Contract (required for race safety)
    /// Must synchronously, in this order:
    /// 1. clear/consume any already-pending edge/event state,
    /// 2. configure the requested edge polarity,
    /// 3. enable/register the event source,
    /// and return only after the detector is ready to observe the NEXT edge.
    /// Step 1 is load-bearing: without it a stale pending interrupt (e.g. a
    /// leftover EXTI flag) makes the subsequent wait return immediately and
    /// the driver reads the conversion register too early. An implementation
    /// that merely records the polarity and defers registration to the first
    /// poll does NOT satisfy this contract: drivers call `arm()` on every RDY
    /// source BEFORE starting any ADC conversion, precisely so a short (~8µs)
    /// conversion-ready pulse cannot occur while no detector is listening.
    fn arm(&mut self, polarity: ConversionReadyPolarity);
    /// Wait for the conversion-ready edge. At the latest, the detector is
    /// armed when the returned future is first polled — but full start-before-
    /// arm safety is only achieved when the driver called [`ConversionReady::arm`]
    /// before starting the conversion (see `wait_all_ready`).
    fn wait_for_ready(&mut self, polarity: ConversionReadyPolarity) -> Self::WaitFuture<'_>;
}

/// Blocking (non-async) conversion-ready signal: the sync counterpart of
/// [`ConversionReady`], for bare-metal / RTIC-style drivers without an executor.
/// Implementations MUST block on a hardware event (GPIO interrupt flag, EXTI
/// wait, etc.), never poll the ADC CONFIG register — OS-bit polling is invalid
/// in continuous mode (OS reads 0 there) and would hang forever.
pub trait BlockingReady {
    type Error;
    /// Synchronously arm the edge detector for `polarity`, returning only once
    /// the detector will observe the next edge. Same contract as
    /// [`ConversionReady::arm`] — including clearing stale pending state
    /// first: lazy/deferred arming reintroduces the start-before-arm race,
    /// and a leftover pending flag causes an immediate bogus wake.
    fn arm(&mut self, polarity: ConversionReadyPolarity);
    /// Block until the conversion-ready edge for `polarity` is observed.
    fn wait_for_ready(&mut self, polarity: ConversionReadyPolarity) -> Result<(), Self::Error>;
}

/// Concurrent wait over N ready sources: ALL `wait_for_ready` futures are
/// created up front and polled round-robin inside ONE future, so every wait
/// observes its edge without sequential-await gaps. This fixes the
/// *sequential-wait* race (`wait(RDY0).await; wait(RDY1).await; ...` missing
/// the ~8µs continuous-mode pulses of ADC1..N while blocked on ADC0).
///
/// NOTE — arming vs. polling: each detector is (at the latest) armed when its
/// future is first polled, i.e. when THIS future is first polled. That alone
/// does NOT protect against pulses that fire between ADC START and the first
/// poll. Drivers must therefore call [`ConversionReady::arm`] on every source
/// BEFORE starting any conversion; `wait_all_ready` then only observes edges
/// on already-armed detectors. (`capture_parallel_event`/`next_all_event` do
/// exactly this: arm all → start → wait all → read all.)
///
/// Returns when every source is ready; on failure reports the failing source
/// index via [`WaitFailed`]. `no_std`, no alloc, no executor-specific join dependency.
#[cfg(feature = "async")]
pub fn wait_all_ready<R: ConversionReady, const N: usize>(
    rdy: &mut [R; N],
    pol: [ConversionReadyPolarity; N],
) -> JoinAllReady<'_, R, N> {    // Disjoint elements via raw pointer: each future borrows only its own RDY.
    let ptr = rdy.as_mut_ptr();
    let futs = core::array::from_fn(|i| unsafe { (&mut *ptr.add(i)).wait_for_ready(pol[i]) });
    JoinAllReady { futs, done: [false; N], remaining: N }
}

/// Wait failure with the source index of the failed RDY future, so callers can
/// attribute the error to the correct device. `no_std`, no alloc.
#[cfg(feature = "async")]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WaitFailed<E> {
    pub index: usize,
    pub error: E,
}

/// Future returned by [`wait_all_ready`]. See its docs for race-safety rationale.
#[cfg(feature = "async")]
pub struct JoinAllReady<'r, R: ConversionReady + 'r, const N: usize> {
    futs: [R::WaitFuture<'r>; N],
    done: [bool; N],
    remaining: usize,
}

#[cfg(feature = "async")]
impl<R: ConversionReady, const N: usize> core::future::Future for JoinAllReady<'_, R, N> {    type Output = Result<(), WaitFailed<R::Error>>;
    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        // SAFETY: futures are never moved after creation; only pinned polls.
        let this = unsafe { self.get_unchecked_mut() };
        let mut i = 0;
        while i < N {
            if !this.done[i] {
                let f = unsafe { core::pin::Pin::new_unchecked(&mut this.futs[i]) };
                match f.poll(cx) {
                    core::task::Poll::Ready(Ok(())) => {
                        this.done[i] = true;
                        this.remaining -= 1;
                    }
                    core::task::Poll::Ready(Err(error)) => {
                        return core::task::Poll::Ready(Err(WaitFailed { index: i, error }))
                    }
                    core::task::Poll::Pending => {}
                }
            }
            i += 1;
        }
        if this.remaining == 0 {
            core::task::Poll::Ready(Ok(()))
        } else {
            core::task::Poll::Pending
        }
    }
}

/// Generic async delay (microseconds) for timed-wait fallback / timeouts.
/// Named-future (GAT) style so no `Send` bound is imposed: embedded executors
/// are not necessarily threaded. Embassy users should prefer the `embassy`
/// feature's first-class timer APIs instead of implementing this manually.
pub trait AsyncDelayUs {
    type DelayFuture<'a>: core::future::Future<Output = ()> + 'a
    where
        Self: 'a;
    fn delay_us(&mut self, us: u32) -> Self::DelayFuture<'_>;
}

// ---------- shared config-byte construction (no duplication) ----------

/// Build CONFIG high/low bytes. `os_start=true` sets OS=1 (single-shot start only).
/// Continuous mode MUST use os_start=false (OS is don't-care; MODE=0 runs freely).
/// Bit layout: OS|MUX[2:0]|PGA[2:0]|MODE | DR[2:0]|COMP_MODE|COMP_POL|COMP_LAT|COMP_QUE[1:0]
fn build_config_bytes(cfg: &Config, mux_bits: u8, os_start: bool, mode: Mode) -> [u8; 2] {
    debug_assert!(!(mode == Mode::Continuous && os_start), "continuous must not set OS=1");
    let os = if os_start { 1u8 << 7 } else { 0 };
    let mode_bit = match mode {
        Mode::SingleShot => 1u8,
        Mode::Continuous => 0u8,
    };
    let hi = os | ((mux_bits & 0x07) << 4) | (cfg.gain as u8) | mode_bit;
    let lo = (cfg.data_rate as u8)
        | (cfg.comp_mode as u8)
        | (cfg.comp_pol as u8)
        | (cfg.comp_lat as u8)
        | (cfg.comp_que as u8);
    [hi, lo]
}

/// ADS1113 has no MUX/PGA/comparator: fixed single channel, gain bits forced 0.
fn build_config_bytes_ads1113(data_rate: DataRate, os_start: bool) -> [u8; 2] {
    let os = if os_start { 1u8 << 7 } else { 0 };
    // Datasheet: ADS1113 config fixed except OS/DR/MODE(=1 single-shot).
    let hi = os | 1u8;
    let lo = data_rate as u8 | 0x03; // COMP_QUE=11b disabled
    [hi, lo]
}

/// ADS1113 continuous config: MODE=0 free-running, OS not set.
/// Only consumed by the Embassy timer path.
#[cfg(feature = "embassy")]
fn build_config_bytes_ads1113_continuous(data_rate: DataRate) -> [u8; 2] {
    [0u8, data_rate as u8 | 0x03]
}

fn write_config<I2C: I2c>(i2c: &mut I2C, addr: u8, bytes: [u8; 2]) -> Result<(), I2C::Error> {
    i2c.write(addr, &[REG_CONFIG, bytes[0], bytes[1]])
}

fn read_conversion<I2C: I2c>(i2c: &mut I2C, addr: u8) -> Result<i16, I2C::Error> {
    let mut data = [0u8; 2];
    i2c.write_read(addr, &[REG_CONVERSION], &mut data)?;
    Ok(i16::from_be_bytes(data))
}

/// Approximate conversion period in microseconds for a data rate (timed-wait fallback).
/// Nominal `1_000_000/SPS` plus a safety margin of ~12.5% + 100µs. This provides
/// a conservative delay intended to allow the conversion to complete under
/// oscillator variation and normal scheduling jitter — deliberately trading a
/// fraction of throughput (at 860 SPS: ~1.41ms wait, ≈710 reads/s max, instead
/// of the nominal 1.16ms). It cannot mathematically guarantee completion on
/// every device under every condition; ALERT/RDY remains the preferred sync
/// method for full-rate capture. Use this fallback only when RDY is unwired.
pub fn conversion_period_us(dr: DataRate) -> u32 {
    let sps = match dr {
        DataRate::SPS8 => 8,
        DataRate::SPS16 => 16,
        DataRate::SPS32 => 32,
        DataRate::SPS64 => 64,
        DataRate::SPS128 => 128,
        DataRate::SPS250 => 250,
        DataRate::SPS475 => 475,
        DataRate::SPS860 => 860,
    };
    let nominal = 1_000_000 / sps as u32;
    nominal + nominal / 8 + 100
}

// ---------- constructors / common accessors ----------

impl<I2C> ADS1113<I2C> {
    pub fn new(i2c: I2C, config: Config) -> Self {
        Self { address: Address::Ground, i2c, config }
    }
    pub fn release(self) -> I2C { self.i2c }
}

impl<I2C> ADS1114<I2C> {
    pub fn new(i2c: I2C, config: Config) -> Self {
        Self { address: Address::Ground, i2c, config }
    }
    pub fn release(self) -> I2C { self.i2c }
}

impl<I2C> ADS1115<I2C> {
    pub fn new(address: Address, i2c: I2C, config: Config) -> Self {
        Self { address, i2c, config }
    }
    pub fn release(self) -> I2C { self.i2c }
}

// ---------- synchronous API (preserved) ----------

/// Blocking OS-bit wait for SINGLE-SHOT mode only (START with OS=1, then OS
/// reads back 1 when done). MUST NOT be used for continuous mode: there OS
/// reads 0 and this would poll forever. Continuous sync paths use
/// [`BlockingReady`] or a caller-supplied delay instead.
fn wait_complete_sync<I2C: I2c>(i2c: &mut I2C, addr: u8) -> Result<(), I2C::Error> {
    let mut buf = [0u8; 2];
    loop {
        i2c.write_read(addr, &[REG_CONFIG], &mut buf)?;
        if conversion_complete(buf[0]) { break; }
    }
    Ok(())
}

impl<I2C: I2c> ADS1113<I2C> {
    /// NOTE: ADS1113 ignores gain/comparator fields in `Config` (no PGA/comparator on HW).
    fn read_channel_raw(&mut self) -> Result<i16, I2C::Error> {
        let b = build_config_bytes_ads1113(self.config.data_rate, true);
        write_config(&mut self.i2c, self.address as u8, b)?;
        wait_complete_sync(&mut self.i2c, self.address as u8)?;
        read_conversion(&mut self.i2c, self.address as u8)
    }
    pub fn read_adc(&mut self) -> Result<i16, I2C::Error> { self.read_channel_raw() }
    /// Timed (non-polling) sync variant: blocks on provided `delay_us` closure.
    pub fn read_adc_timed(&mut self, mut delay_us: impl FnMut(u32)) -> Result<i16, I2C::Error> {
        let b = build_config_bytes_ads1113(self.config.data_rate, true);
        write_config(&mut self.i2c, self.address as u8, b)?;
        delay_us(conversion_period_us(self.config.data_rate));
        read_conversion(&mut self.i2c, self.address as u8)
    }
}

impl<I2C: I2c> ADS1114<I2C> {
    /// Configure ALERT/RDY as conversion-ready: writes HI=0x8000/LO=0x0000
    /// AND upgrades `self.config` via `as_conversion_ready()` (COMP_QUE=
    /// OneConversion, NonLatching), because the default `DisableComparator`
    /// (11b) would silently keep the pin inactive. Subsequent starts therefore
    /// emit a usable conversion-ready signal.
    ///
    /// If an I2C error occurs mid-setup (HI written, LO not), the hardware may
    /// hold a partially modified threshold pair while `self.config` remains
    /// unchanged — a safe software state, but retry/reinitialize the registers
    /// before relying on RDY rather than assuming either half took effect.
    pub fn enable_conversion_ready_pin(&mut self) -> Result<(), I2C::Error> {
        enable_rdy_registers(&mut self.i2c, self.address as u8)?;
        self.config = self.config.as_conversion_ready();
        Ok(())
    }
    /// A. start conversion (OS=1); returns immediately.
    pub fn start_conversion(&mut self) -> Result<(), I2C::Error> {
        let b = build_config_bytes(&self.config, 0b100, true, Mode::SingleShot);
        write_config(&mut self.i2c, self.address as u8, b)
    }
    /// C. read conversion register (call after READY observed).
    pub fn read_conversion(&mut self) -> Result<i16, I2C::Error> {
        read_conversion(&mut self.i2c, self.address as u8)
    }
    /// Continuous mode: start + streaming reads.
    /// MODE=0, OS NOT set. First sample valid only after RDY/timed wait.
    /// (Sync names are kept stable; like the async `_unchecked` starters, this
    /// does not arm RDY — arm the detector before starting for event-driven use.)
    pub fn start_continuous(&mut self) -> Result<(), I2C::Error> {
        let b = build_config_bytes(&self.config, 0b100, false, Mode::Continuous);
        write_config(&mut self.i2c, self.address as u8, b)
    }
    /// Next sample of an already-running continuous stream via a blocking RDY
    /// wait: re-arm, then wait, then read. No restart, no CONFIG polling (OS
    /// polling is invalid here: OS reads 0 in continuous mode). Returns the
    /// latest sample (no FIFO).
    ///
    /// # Precondition
    /// Conversion-ready mode enabled (`enable_conversion_ready_pin()`).
    pub fn next_conversion_blocking<R: BlockingReady>(
        &mut self,
        rdy: &mut R,
    ) -> Result<i16, Error<I2C::Error, R::Error>> {
        debug_assert_rdy(&self.config, "ADS1114::next_conversion_blocking");
        let pol = ConversionReadyPolarity::from(self.config.comp_pol);
        rdy.arm(pol);
        rdy.wait_for_ready(pol).map_err(Error::Wait)?;
        self.read_conversion().map_err(Error::I2c)
    }
    /// Next continuous sample via a caller-supplied delay (no RDY wiring
    /// needed): waits ~one conversion period, then reads the LATEST conversion.
    /// Not "the conversion from exactly one period ago": the ADC free-runs and
    /// I2C traffic shifts actual sample instants. No CONFIG polling.
    pub fn next_conversion_timed(
        &mut self,
        mut delay_us: impl FnMut(u32),
    ) -> Result<i16, I2C::Error> {
        delay_us(conversion_period_us(self.config.data_rate));
        self.read_conversion()
    }
    fn read_channel_raw(&mut self) -> Result<i16, I2C::Error> {
        self.start_conversion()?;
        wait_complete_sync(&mut self.i2c, self.address as u8)?;
        self.read_conversion()
    }
    pub fn read_adc(&mut self) -> Result<i16, I2C::Error> { self.read_channel_raw() }
}

/// Signed raw -> (negative, magnitude). false=positive (A+>A-), true=negative.
/// Uses unsigned_abs: handles i16::MIN correctly (no abs() overflow).
#[inline]
pub fn signed_to_polarity_magnitude(value: i16) -> (bool, u16) {
    if value < 0 { (true, value.unsigned_abs()) } else { (false, value as u16) }
}

/// Zero-cost single-ended channel storage (one ADC+MUX: values filled sequentially).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Channel0(pub u16);
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Channel1(pub u16);
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Channel2(pub u16);
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Channel3(pub u16);

fn enable_rdy_registers<I2C: I2c>(i2c: &mut I2C, addr: u8) -> Result<(), I2C::Error> {
    // Hi_thresh MSB=1, Lo_thresh MSB=0; requires COMP_QUE != 11b (checked by caller docs).
    i2c.write(addr, &[REG_HI_THRESH, 0x80, 0x00])?;
    i2c.write(addr, &[REG_LO_THRESH, 0x00, 0x00])?;
    Ok(())
}

impl<I2C: I2c> ADS1115<I2C> {
    /// Same as ADS1114: threshold registers plus `self.config` upgrade so the
    /// pin is actually usable (see above, including the partial-write note:
    /// on I2C error retry/reinitialize before relying on RDY).
    pub fn enable_conversion_ready_pin(&mut self) -> Result<(), I2C::Error> {
        enable_rdy_registers(&mut self.i2c, self.address as u8)?;
        self.config = self.config.as_conversion_ready();
        Ok(())
    }
    pub fn start_conversion(&mut self, mux: Mux) -> Result<(), I2C::Error> {
        let b = build_config_bytes(&self.config, mux as u8, true, Mode::SingleShot);
        write_config(&mut self.i2c, self.address as u8, b)
    }
    pub fn read_conversion(&mut self) -> Result<i16, I2C::Error> {
        read_conversion(&mut self.i2c, self.address as u8)
    }
    pub fn start_continuous(&mut self, mux: Mux) -> Result<(), I2C::Error> {
        // NOTE: does not arm RDY (sync names kept stable); arm the detector
        // before starting for event-driven `next_conversion_blocking()` use.
        let b = build_config_bytes(&self.config, mux as u8, false, Mode::Continuous);
        write_config(&mut self.i2c, self.address as u8, b)
    }
    /// Next continuous sample via a blocking RDY wait: re-arm, then wait, then
    /// read. No restart, no CONFIG polling (OS polling is invalid here: OS
    /// reads 0 in continuous mode). Returns the latest sample (no FIFO).
    ///
    /// # Precondition
    /// Conversion-ready mode enabled (`enable_conversion_ready_pin()`).
    pub fn next_conversion_blocking<R: BlockingReady>(
        &mut self,
        rdy: &mut R,
    ) -> Result<i16, Error<I2C::Error, R::Error>> {
        debug_assert_rdy(&self.config, "ADS1115::next_conversion_blocking");
        let pol = ConversionReadyPolarity::from(self.config.comp_pol);
        rdy.arm(pol);
        rdy.wait_for_ready(pol).map_err(Error::Wait)?;
        self.read_conversion().map_err(Error::I2c)
    }
    /// Next continuous sample via a caller-supplied delay (no RDY wiring
    /// needed): waits ~one conversion period, then reads the LATEST conversion.
    /// Not "the conversion from exactly one period ago": the ADC free-runs and
    /// I2C traffic shifts actual sample instants. No CONFIG polling.
    pub fn next_conversion_timed(
        &mut self,
        mut delay_us: impl FnMut(u32),
    ) -> Result<i16, I2C::Error> {
        delay_us(conversion_period_us(self.config.data_rate));
        self.read_conversion()
    }
    fn read_channel_raw(&mut self, mux: Mux) -> Result<i16, I2C::Error> {
        // Exclusive &mut self from start->read guarantees no stale-channel hazard.
        self.start_conversion(mux)?;
        wait_complete_sync(&mut self.i2c, self.address as u8)?;
        self.read_conversion()
    }
    /// Typed MUX API (preferred). Raw-bits variant validates range (no silent masking).
    pub fn read_channel(&mut self, mux: Mux) -> Result<i16, I2C::Error> { self.read_channel_raw(mux) }
    pub fn read_channel_raw_bits(&mut self, mux: u8) -> Result<i16, MuxOrI2c<I2C::Error>> {
        Ok(self.read_channel_raw(match_mux(mux).map_err(|_| MuxOrI2c::Mux(MuxError(mux)))?).map_err(MuxOrI2c::I2c)?)
    }
    pub fn read_adc_a0(&mut self) -> Result<i16, I2C::Error> { self.read_channel_raw(Mux::A0) }
    pub fn read_adc_a1(&mut self) -> Result<i16, I2C::Error> { self.read_channel_raw(Mux::A1) }
    pub fn read_adc_a2(&mut self) -> Result<i16, I2C::Error> { self.read_channel_raw(Mux::A2) }
    pub fn read_adc_a3(&mut self) -> Result<i16, I2C::Error> { self.read_channel_raw(Mux::A3) }
    pub fn read_adc_a0n1(&mut self) -> Result<i16, I2C::Error> { self.read_channel_raw(Mux::A0N1) }
    pub fn read_adc_a0n3(&mut self) -> Result<i16, I2C::Error> { self.read_channel_raw(Mux::A0N3) }
    pub fn read_adc_a1n3(&mut self) -> Result<i16, I2C::Error> { self.read_channel_raw(Mux::A1N3) }
    pub fn read_adc_a2n3(&mut self) -> Result<i16, I2C::Error> { self.read_channel_raw(Mux::A2N3) }
    /// Sequential single-shot conversions (single ADC + MUX: cannot overlap).
    pub fn read_4adc(&mut self) -> Result<[i16; 4], I2C::Error> {
        Ok([self.read_adc_a0()?, self.read_adc_a1()?, self.read_adc_a2()?, self.read_adc_a3()?])
    }
    /// Typed single-ended channel structs (sequential fills; NOT simultaneous).
    pub fn read_channels(&mut self) -> Result<(Channel0, Channel1, Channel2, Channel3), I2C::Error> {
        Ok((Channel0(self.read_adc_a0()? as u16), Channel1(self.read_adc_a1()? as u16),
            Channel2(self.read_adc_a2()? as u16), Channel3(self.read_adc_a3()? as u16)))
    }
    // --- Differential pairs (one converter + MUX; NOT simultaneous sampling) ---
    // Only HW-valid pairs exposed; reverse polarity = sign of result.
    pub fn read_a0_a1(&mut self) -> Result<(bool, u16), I2C::Error> {
        Ok(signed_to_polarity_magnitude(self.read_channel_raw(Mux::A0N1)?))
    }
    pub fn read_a0_a3(&mut self) -> Result<(bool, u16), I2C::Error> {
        Ok(signed_to_polarity_magnitude(self.read_channel_raw(Mux::A0N3)?))
    }
    pub fn read_a1_a3(&mut self) -> Result<(bool, u16), I2C::Error> {
        Ok(signed_to_polarity_magnitude(self.read_channel_raw(Mux::A1N3)?))
    }
    pub fn read_a2_a3(&mut self) -> Result<(bool, u16), I2C::Error> {
        Ok(signed_to_polarity_magnitude(self.read_channel_raw(Mux::A2N3)?))
    }
}

/// Rejected raw MUX value (valid: 0..=7 mapping to the 8 ADS1115 selections).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MuxError(pub u8);

/// Error wrapper for the compat raw-bits helper only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MuxOrI2c<E> {
    Mux(MuxError),
    I2c(E),
}

fn match_mux(bits: u8) -> Result<Mux, MuxError> {
    match bits {
        0 => Ok(Mux::A0N1), 1 => Ok(Mux::A0N3), 2 => Ok(Mux::A1N3), 3 => Ok(Mux::A2N3),
        4 => Ok(Mux::A0), 5 => Ok(Mux::A1), 6 => Ok(Mux::A2), 7 => Ok(Mux::A3),
        b => Err(MuxError(b)),
    }
}

// ---------- async API (event-driven, no polling) ----------

#[cfg(feature = "async")]
mod async_api {
    use super::*;
    use embedded_hal_async::i2c::I2c as AsyncI2c;

    async fn write_config_async<I2C: AsyncI2c>(i2c: &mut I2C, addr: u8, b: [u8; 2]) -> Result<(), I2C::Error> {
        i2c.write(addr, &[REG_CONFIG, b[0], b[1]]).await
    }
    async fn read_conv_async<I2C: AsyncI2c>(i2c: &mut I2C, addr: u8) -> Result<i16, I2C::Error> {
        let mut d = [0u8; 2];
        i2c.write_read(addr, &[REG_CONVERSION], &mut d).await?;
        Ok(i16::from_be_bytes(d))
    }

    impl<I2C: AsyncI2c> ADS1113<I2C> {
        /// ADS1113 has NO ALERT/RDY: timed async wait only (explicit model difference).
        pub async fn read_adc_async<D: AsyncDelayUs>(&mut self, delay: &mut D) -> Result<i16, I2C::Error> {
            self.start_conversion_async().await?;
            delay.delay_us(conversion_period_us(self.config.data_rate)).await;
            self.read_conversion_async().await
        }
        pub async fn start_conversion_async(&mut self) -> Result<(), I2C::Error> {
            let b = build_config_bytes_ads1113(self.config.data_rate, true);
            write_config_async(&mut self.i2c, self.address as u8, b).await
        }
        pub async fn read_conversion_async(&mut self) -> Result<i16, I2C::Error> {
            read_conv_async(&mut self.i2c, self.address as u8).await
        }
    }

    impl<I2C: AsyncI2c> ADS1114<I2C> {
        /// Async RDY-register setup (same bytes as sync; async only because I2C is async).
        /// On I2C error the hardware may hold a partial threshold pair while
        /// `self.config` is unchanged — retry/reinitialize before relying on RDY.
        pub async fn enable_conversion_ready_pin_async(&mut self) -> Result<(), I2C::Error> {
            self.i2c.write(self.address as u8, &[REG_HI_THRESH, 0x80, 0x00]).await?;
            self.i2c.write(self.address as u8, &[REG_LO_THRESH, 0x00, 0x00]).await?;
            // Same comparator upgrade as the sync helper: without COMP_QUE !=
            // 11b the pin would stay inactive despite the thresholds.
            self.config = self.config.as_conversion_ready();
            Ok(())
        }
        pub async fn start_conversion_async(&mut self) -> Result<(), I2C::Error> {
            let b = build_config_bytes(&self.config, 0b100, true, Mode::SingleShot);
            write_config_async(&mut self.i2c, self.address as u8, b).await
        }
        pub async fn start_continuous_async_unchecked(&mut self) -> Result<(), I2C::Error> {
            // MODE=0 runs freely; OS=1 NOT used per conversion.
            // UNCHECKED: the caller MUST have armed RDY via `rdy.arm()` BEFORE
            // calling this (see `continuous()`); the name is the warning.
            let b = build_config_bytes(&self.config, 0b100, false, Mode::Continuous);
            write_config_async(&mut self.i2c, self.address as u8, b).await
        }
        pub async fn read_conversion_async(&mut self) -> Result<i16, I2C::Error> {
            read_conv_async(&mut self.i2c, self.address as u8).await
        }
        /// Preferred Embassy flow. Arm RDY BEFORE start (per `ConversionReady`
        /// contract) so the ~8µs pulse cannot fire while unlistened; `&mut self`
        /// held throughout so no other op can interleave (no stale-data hazard).
        /// Documented ordering: enable RDY regs -> arm -> start -> wait edge -> read.
        ///
        /// # Precondition
        /// Conversion-ready mode must be enabled first (`enable_conversion_ready_pin_async()`,
        /// which also sets `COMP_QUE != DisableComparator`). Otherwise ALERT/RDY
        /// stays inactive and this wait hangs forever (debug builds assert this).
        pub async fn read_adc_event<R: ConversionReady>(&mut self, rdy: &mut R) -> Result<i16, Error<I2C::Error, R::Error>> {
            debug_assert_rdy(&self.config, "ADS1114::read_adc_event");
            let pol = ConversionReadyPolarity::from(self.config.comp_pol);
            rdy.arm(pol);
            self.start_conversion_async().await.map_err(Error::I2c)?;
            rdy.wait_for_ready(pol).await.map_err(Error::Wait)?;
            self.read_conversion_async().await.map_err(Error::I2c)
        }
        /// Fallback timed wait (documented: less precise than ALERT/RDY).
        pub async fn read_adc_timed<D: AsyncDelayUs>(&mut self, delay: &mut D) -> Result<i16, I2C::Error> {
            self.start_conversion_async().await?;
            delay.delay_us(conversion_period_us(self.config.data_rate)).await;
            self.read_conversion_async().await
        }
        /// Next sample of a RUNNING continuous stream: re-arm RDY, wait, read.
        /// Re-arming every cycle is required: continuous RDY is a ~8µs pulse
        /// that may fire between the previous read and this wait. No restart.
        ///
        /// # Precondition
        /// Conversion-ready mode enabled (established by `continuous()` or
        /// `enable_conversion_ready_pin_async()`).
        pub async fn next_conversion<R: ConversionReady>(&mut self, rdy: &mut R) -> Result<i16, Error<I2C::Error, R::Error>> {
            debug_assert_rdy(&self.config, "ADS1114::next_conversion");
            let pol = ConversionReadyPolarity::from(self.config.comp_pol);
            rdy.arm(pol);
            rdy.wait_for_ready(pol).await.map_err(Error::Wait)?;
            self.read_conversion_async().await.map_err(Error::I2c)
        }
        /// Race-safe continuous stream constructor (ADS1114 has a single input
        /// path, so no MUX argument): arms `rdy` BEFORE starting the ADC so the
        /// first ~8µs pulse cannot be missed. `stream.next()` re-arms every cycle.
        ///
        /// # Precondition
        /// Conversion-ready mode must be enabled first (`enable_conversion_ready_pin_async()`).
        pub async fn continuous<'a, 'r, R: ConversionReady>(&'a mut self, rdy: &'r mut R) -> Result<Continuous<'a, 'r, I2C, R, 4>, I2C::Error> {
            debug_assert_rdy(&self.config, "ADS1114::continuous");
            rdy.arm(ConversionReadyPolarity::from(self.config.comp_pol));
            self.start_continuous_async_unchecked().await?;
            Ok(Continuous { adc: self, rdy })
        }
    }

    impl<I2C: AsyncI2c> ADS1115<I2C> {
        /// Async RDY-register setup (same contract as the sync helper,
        /// including the partial-write note: retry/reinitialize on I2C error).
        pub async fn enable_conversion_ready_pin_async(&mut self) -> Result<(), I2C::Error> {
            self.i2c.write(self.address as u8, &[REG_HI_THRESH, 0x80, 0x00]).await?;
            self.i2c.write(self.address as u8, &[REG_LO_THRESH, 0x00, 0x00]).await?;
            // Comparator upgrade (see sync helper): thresholds alone are inert.
            self.config = self.config.as_conversion_ready();
            Ok(())
        }
        pub async fn start_conversion_async(&mut self, mux: Mux) -> Result<(), I2C::Error> {
            let b = build_config_bytes(&self.config, mux as u8, true, Mode::SingleShot);
            write_config_async(&mut self.i2c, self.address as u8, b).await
        }
        pub async fn start_continuous_async_unchecked(&mut self, mux: Mux) -> Result<(), I2C::Error> {
            // MODE=0 free-running; OS not re-asserted per sample.
            // UNCHECKED: the caller MUST have armed RDY via `rdy.arm()` BEFORE
            // calling this — the name is the warning. Prefer the race-safe
            // `continuous()` constructor for event-driven use.
            let b = build_config_bytes(&self.config, mux as u8, false, Mode::Continuous);
            write_config_async(&mut self.i2c, self.address as u8, b).await
        }
        pub async fn read_conversion_async(&mut self) -> Result<i16, I2C::Error> {
            read_conv_async(&mut self.i2c, self.address as u8).await
        }
        /// arm -> start -> await RDY edge -> read. `&mut self` throughout: no interleave hazard.
        ///
        /// # Precondition
        /// Conversion-ready mode must be enabled first (`enable_conversion_ready_pin_async()`).
        /// Otherwise ALERT/RDY stays inactive and this wait hangs forever (debug builds assert this).
        pub async fn read_channel_event<R: ConversionReady>(&mut self, mux: Mux, rdy: &mut R) -> Result<i16, Error<I2C::Error, R::Error>> {
            debug_assert_rdy(&self.config, "ADS1115::read_channel_event");
            let pol = ConversionReadyPolarity::from(self.config.comp_pol);
            rdy.arm(pol);
            self.start_conversion_async(mux).await.map_err(Error::I2c)?;
            rdy.wait_for_ready(pol).await.map_err(Error::Wait)?;
            self.read_conversion_async().await.map_err(Error::I2c)
        }
        /// Timed fallback: start -> delay(dr) -> read. No config polling.
        pub async fn read_channel_timed<D: AsyncDelayUs>(&mut self, mux: Mux, delay: &mut D) -> Result<i16, I2C::Error> {
            self.start_conversion_async(mux).await?;
            delay.delay_us(conversion_period_us(self.config.data_rate)).await;
            self.read_conversion_async().await
        }
        // Idiomatic per-channel async wrappers (event-driven).
        pub async fn read_adc_a0_async<R: ConversionReady>(&mut self, rdy: &mut R) -> Result<i16, Error<I2C::Error, R::Error>> { self.read_channel_event(Mux::A0, rdy).await }
        pub async fn read_adc_a1_async<R: ConversionReady>(&mut self, rdy: &mut R) -> Result<i16, Error<I2C::Error, R::Error>> { self.read_channel_event(Mux::A1, rdy).await }
        pub async fn read_adc_a2_async<R: ConversionReady>(&mut self, rdy: &mut R) -> Result<i16, Error<I2C::Error, R::Error>> { self.read_channel_event(Mux::A2, rdy).await }
        pub async fn read_adc_a3_async<R: ConversionReady>(&mut self, rdy: &mut R) -> Result<i16, Error<I2C::Error, R::Error>> { self.read_channel_event(Mux::A3, rdy).await }
        /// Sequential multi-channel event reads (single converter: strictly sequential).
        pub async fn read_4adc_event<R: ConversionReady>(&mut self, rdy: &mut R) -> Result<[i16; 4], Error<I2C::Error, R::Error>> {
            Ok([self.read_channel_event(Mux::A0, rdy).await?, self.read_channel_event(Mux::A1, rdy).await?,
                self.read_channel_event(Mux::A2, rdy).await?, self.read_channel_event(Mux::A3, rdy).await?])
        }
        pub async fn read_4adc_timed<D: AsyncDelayUs>(&mut self, delay: &mut D) -> Result<[i16; 4], I2C::Error> {
            Ok([self.read_channel_timed(Mux::A0, delay).await?, self.read_channel_timed(Mux::A1, delay).await?,
                self.read_channel_timed(Mux::A2, delay).await?, self.read_channel_timed(Mux::A3, delay).await?])
        }
        // --- Differential pairs (one converter + MUX, sequential; NOT simultaneous) ---
        async fn read_diff_event<R: ConversionReady>(&mut self, mux: Mux, rdy: &mut R) -> Result<(bool, u16), Error<I2C::Error, R::Error>> {
            Ok(signed_to_polarity_magnitude(self.read_channel_event(mux, rdy).await?))
        }
        pub async fn read_a0_a1_async<R: ConversionReady>(&mut self, rdy: &mut R) -> Result<(bool, u16), Error<I2C::Error, R::Error>> { self.read_diff_event(Mux::A0N1, rdy).await }
        pub async fn read_a0_a3_async<R: ConversionReady>(&mut self, rdy: &mut R) -> Result<(bool, u16), Error<I2C::Error, R::Error>> { self.read_diff_event(Mux::A0N3, rdy).await }
        pub async fn read_a1_a3_async<R: ConversionReady>(&mut self, rdy: &mut R) -> Result<(bool, u16), Error<I2C::Error, R::Error>> { self.read_diff_event(Mux::A1N3, rdy).await }
        pub async fn read_a2_a3_async<R: ConversionReady>(&mut self, rdy: &mut R) -> Result<(bool, u16), Error<I2C::Error, R::Error>> { self.read_diff_event(Mux::A2N3, rdy).await }
        // --- Continuous: start once, then next_* waits RDY + reads (no restart) ---
        // PULSE SAFETY: continuous RDY is ~8us pulse. The race-safe entry point
        // is `continuous()`, which arms RDY BEFORE starting the ADC. The raw
        // `start_continuous_async_unchecked()` below is deliberately
        // scary-named: callers using it directly MUST call `rdy.arm(polarity)`
        // themselves first, otherwise the first pulse can fire while
        // unlistened. Requires conversion-ready mode (see `continuous()`);
        // debug builds assert it.
        pub async fn next_conversion<R: ConversionReady>(&mut self, rdy: &mut R) -> Result<i16, Error<I2C::Error, R::Error>> {
            debug_assert_rdy(&self.config, "ADS1115::next_conversion");
            let pol = ConversionReadyPolarity::from(self.config.comp_pol);
            rdy.arm(pol);
            rdy.wait_for_ready(pol).await.map_err(Error::Wait)?;
            self.read_conversion_async().await.map_err(Error::I2c)
        }
        /// Race-safe stream constructor: arms `rdy` BEFORE starting the ADC so the
        /// first ~8us pulse cannot be missed. Prefer this over manual
        /// `start_continuous_async_unchecked()` + late `wait_for_ready()`.
        ///
        /// # Precondition
        /// Conversion-ready mode must be enabled first (`enable_conversion_ready_pin_async()`).
        pub async fn continuous<'a, 'r, R: ConversionReady>(&'a mut self, mux: Mux, rdy: &'r mut R) -> Result<Continuous<'a, 'r, I2C, R, 5>, I2C::Error> {
            debug_assert_rdy(&self.config, "ADS1115::continuous");
            rdy.arm(ConversionReadyPolarity::from(self.config.comp_pol));
            self.start_continuous_async_unchecked(mux).await?;
            Ok(Continuous { adc: self, rdy })
        }
    }

    /// Borrowed continuous stream: ADC started once; `next()` re-arms RDY,
    /// awaits the pulse, and reads. Holds `&mut` borrows so no other op can
    /// interleave (no stale-data hazard). `no_std`, no alloc, no
    /// futures::Stream dependency. Generic over the device model so ADS1114
    /// and ADS1115 share the race-safe constructor/stream API.
    pub struct Continuous<'a, 'r, I2C, RDY, const MODEL: u8> {
        adc: &'a mut ADS111x<I2C, MODEL>,
        rdy: &'r mut RDY,
    }

    impl<I2C: AsyncI2c, RDY: ConversionReady> Continuous<'_, '_, I2C, RDY, 5> {
        /// Next raw sample of the running stream (re-arm → wait → read).
        /// The MUX was fixed at `continuous()` time, so differential results
        /// are the caller's one-line conversion, e.g.:
        /// `signed_to_polarity_magnitude(stream.next().await?)`.
        /// (Per-MUX `next_a*_a*()` shorthands were removed: they could not
        /// select a MUX and misled readers of streams started on another pair.)
        pub async fn next(&mut self) -> Result<i16, Error<I2C::Error, RDY::Error>> {
            self.adc.next_conversion(self.rdy).await
        }
    }

    impl<I2C: AsyncI2c, RDY: ConversionReady> Continuous<'_, '_, I2C, RDY, 4> {
        /// Next ADS1114 continuous sample (re-arm → wait → read; ADS1114 has a
        /// single input path, so no MUX/differential variants).
        pub async fn next(&mut self) -> Result<i16, Error<I2C::Error, RDY::Error>> {
            self.adc.next_conversion(self.rdy).await
        }
    }

    // ---------- Embassy Timer fallback (feature "embassy", no RDY wiring needed) ----------
    // start -> Timer::after(nominal period + margin) -> read. No CONFIG polling,
    // no GPIO, timer returns no error so Result<T, I2C::Error> suffices.
    #[cfg(feature = "embassy")]
    async fn timer_wait_us(us: u32) {
        embassy_time::Timer::after(embassy_time::Duration::from_micros(us as u64)).await
    }

    #[cfg(feature = "embassy")]
    impl<I2C: AsyncI2c> ADS1113<I2C> {
        /// Single-shot via Embassy Timer (ADS1113 has no ALERT/RDY).
        pub async fn read_adc_timer(&mut self) -> Result<i16, I2C::Error> {
            self.start_conversion_async().await?;
            timer_wait_us(conversion_period_us(self.config.data_rate)).await;
            self.read_conversion_async().await
        }
        /// Continuous: MODE=0 free-running (MODE=1 would be single-shot).
        /// ADS1113 has no ALERT/RDY, so this timer path starts the ADC once,
        pub async fn start_continuous_timer(&mut self) -> Result<(), I2C::Error> {
            let b = build_config_bytes_ads1113_continuous(self.config.data_rate);
            write_config_async(&mut self.i2c, self.address as u8, b).await
        }
        pub async fn next_timer(&mut self) -> Result<i16, I2C::Error> {
            timer_wait_us(conversion_period_us(self.config.data_rate)).await;
            self.read_conversion_async().await
        }
    }

    #[cfg(feature = "embassy")]
    impl<I2C: AsyncI2c> ADS1114<I2C> {
        pub async fn read_adc_timer(&mut self) -> Result<i16, I2C::Error> {
            self.start_conversion_async().await?;
            timer_wait_us(conversion_period_us(self.config.data_rate)).await;
            self.read_conversion_async().await
        }
        pub async fn continuous_timer(&mut self) -> Result<ContinuousTimer<'_, I2C, 4>, I2C::Error> {
            let dr = self.config.data_rate;
            self.start_continuous_async_unchecked().await?;
            // First sample valid only after one full period; `next()` waits before each read.
            Ok(ContinuousTimer { adc: self, period_us: conversion_period_us(dr) })
        }
    }

    #[cfg(feature = "embassy")]
    impl<I2C: AsyncI2c> ADS1115<I2C> {
        /// Single-shot via Embassy Timer: no timer argument, no RDY needed.
        pub async fn read_channel_timer(&mut self, mux: Mux) -> Result<i16, I2C::Error> {
            self.start_conversion_async(mux).await?;
            timer_wait_us(conversion_period_us(self.config.data_rate)).await;
            self.read_conversion_async().await
        }
        pub async fn read_adc_a0_timer(&mut self) -> Result<i16, I2C::Error> { self.read_channel_timer(Mux::A0).await }
        pub async fn read_adc_a1_timer(&mut self) -> Result<i16, I2C::Error> { self.read_channel_timer(Mux::A1).await }
        pub async fn read_adc_a2_timer(&mut self) -> Result<i16, I2C::Error> { self.read_channel_timer(Mux::A2).await }
        pub async fn read_adc_a3_timer(&mut self) -> Result<i16, I2C::Error> { self.read_channel_timer(Mux::A3).await }
        async fn read_diff_timer(&mut self, mux: Mux) -> Result<(bool, u16), I2C::Error> {
            Ok(signed_to_polarity_magnitude(self.read_channel_timer(mux).await?))
        }
        pub async fn read_a0_a1_timer(&mut self) -> Result<(bool, u16), I2C::Error> { self.read_diff_timer(Mux::A0N1).await }
        pub async fn read_a0_a3_timer(&mut self) -> Result<(bool, u16), I2C::Error> { self.read_diff_timer(Mux::A0N3).await }
        pub async fn read_a1_a3_timer(&mut self) -> Result<(bool, u16), I2C::Error> { self.read_diff_timer(Mux::A1N3).await }
        pub async fn read_a2_a3_timer(&mut self) -> Result<(bool, u16), I2C::Error> { self.read_diff_timer(Mux::A2N3).await }
        /// Timer stream: start once (MODE=0, no OS per sample), then
        /// Timer::after(period) + read the LATEST conversion per sample — not
        /// "the conversion from exactly one period ago". No double-delay:
        /// constructor only starts; each `next()` waits ~one period, then reads.
        pub async fn continuous_timer(&mut self, mux: Mux) -> Result<ContinuousTimer<'_, I2C, 5>, I2C::Error> {
            let dr = self.config.data_rate;
            self.start_continuous_async_unchecked(mux).await?;
            Ok(ContinuousTimer { adc: self, period_us: conversion_period_us(dr) })
        }
    }

    /// Timer-based continuous stream (no RDY race: timer needs no arming).
    #[cfg(feature = "embassy")]
    pub struct ContinuousTimer<'a, I2C, const MODEL: u8> {
        adc: &'a mut ADS111x<I2C, MODEL>,
        period_us: u32,
    }

    #[cfg(feature = "embassy")]
    impl<I2C: AsyncI2c> ContinuousTimer<'_, I2C, 5> {
        /// Next raw sample: wait ~one period, then read the LATEST conversion
        /// (not "the conversion from exactly one period ago" — the ADC
        /// free-runs and read timing shifts actual sample instants).
        /// Differential decoding is the caller's
        /// `signed_to_polarity_magnitude(...)` for the MUX selected
        /// at `continuous_timer()` time.
        pub async fn next(&mut self) -> Result<i16, I2C::Error> {
            timer_wait_us(self.period_us).await;
            self.adc.read_conversion_async().await
        }
    }

    #[cfg(feature = "embassy")]
    impl<I2C: AsyncI2c> ContinuousTimer<'_, I2C, 4> {
        pub async fn next(&mut self) -> Result<i16, I2C::Error> {
            timer_wait_us(self.period_us).await;
            self.adc.read_conversion_async().await
        }
    }
}

/// Embassy integration (no core dependency on embassy crates):
/// ```ignore
/// use ads111x_rs::{ConversionReady, ConversionReadyPolarity, AsyncDelayUs};
/// use core::future::Future;
/// struct GpioRdy(embassy_stm32::gpio::Input<'static>);
/// impl ConversionReady for GpioRdy {
///     type Error = core::convert::Infallible;
///     // Named future type: the future only *observes* the edge; synchronous
///     // arming below is what guarantees no pulse fires while unlistened.
///     // `arm()` MUST synchronously clear stale state, configure the edge, and
///     // enable the detector — in that order — returning only once the
///     // detector will observe the NEXT edge, e.g.:
///     // self.0.clear_pending_event(); // consume leftover EXTI flag first
///     // self.0.configure_edge(...);   // then select falling/rising
///     // self.0.enable_interrupt();    // then enable; returns after HW write
///     type WaitFuture<'a> = impl Future<Output = Result<(), Self::Error>> + 'a;
///     fn arm(&mut self, pol: ConversionReadyPolarity) {
///         // 1. Clear stale state FIRST: a leftover pending flag would make the
///         //    subsequent wait return immediately (bogus early read).
///         self.0.clear_pending_event();
///         // 2. Configure the requested edge polarity.
///         // 3. Enable/register the detector (synchronously, before return).
///         match pol {
///             ConversionReadyPolarity::ActiveLow => self.0.enable_falling_edge_interrupt(),
///             ConversionReadyPolarity::ActiveHigh => self.0.enable_rising_edge_interrupt(),
///         }
///         // NOTE: a lazy `arm()` that defers to first poll violates the trait
///         // contract and reintroduces the start-before-arm race. Exact method
///         // names depend on the Embassy HAL (stm32/nRF/RPxxx GPIO/EXTI API);
///         // the clear → configure → enable ORDER is what the contract requires.
///     }
///     fn wait_for_ready(&mut self, pol: ConversionReadyPolarity) -> Self::WaitFuture<'_> {
///         async move {
///             match pol {
///                 ConversionReadyPolarity::ActiveLow => self.0.wait_for_falling_edge().await,
///                 ConversionReadyPolarity::ActiveHigh => self.0.wait_for_rising_edge().await,
///             }
///             Ok(())
///         }
///     }
/// }
/// struct EmbTimer;
/// impl AsyncDelayUs for EmbTimer {
///     type DelayFuture<'a> = impl core::future::Future<Output = ()> + 'a;
///     fn delay_us(&mut self, us: u32) -> Self::DelayFuture<'_> {
///         async move { embassy_time::Timer::after_micros(us).await; }
///     }
/// }
/// // App: wrap RDY wait in embassy_time::with_timeout to avoid hanging forever.
/// ```
/// Cancellation/timeout lives at the app layer (`with_timeout`); core stays timer-free.

/// Electrical safety: PGA full-scale (e.g. `Gain::V6_144`) is a measurement range,
/// NOT permission to exceed supply rails / absolute input limits in the datasheet.
/// Always keep inputs within device absolute maximum ratings.

/**
## Config
`gain`/`comp_*` fields apply only to ADS1114/ADS1115; ADS1113 ignores them (no PGA/comparator).
`comp_que` must not be `DisableComparator` for ALERT/RDY conversion-ready mode.
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
    pub fn with_gain(self, gain: Gain) -> Self { Self { gain, ..self } }
    pub fn with_data_rate(self, data_rate: DataRate) -> Self { Self { data_rate, ..self } }
    pub fn with_comp_mode(self, comp_mode: CompMode) -> Self { Self { comp_mode, ..self } }
    pub fn with_comp_pol(self, comp_pol: CompPol) -> Self { Self { comp_pol, ..self } }
    pub fn with_comp_lat(self, comp_lat: CompLat) -> Self { Self { comp_lat, ..self } }
    pub fn with_comp_que(self, comp_que: CompQue) -> Self { Self { comp_que, ..self } }
    /// Returns a `Config` with comparator settings suitable for RDY
    /// (`COMP_QUE = OneConversion`, non-latching). This is a pure config
    /// transformation: it does NOT program the ADS111x threshold registers
    /// (`HI_THRESH` MSB=1 / `LO_THRESH` MSB=0 still required on hardware).
    /// Call `enable_conversion_ready_pin*()` when configuring hardware — it
    /// writes the registers AND applies this transformation. Relying on reset
    /// threshold values alone is fragile if anything previously rewrote them.
    ///
    /// Kept public deliberately for low-level configurability (e.g. composing
    /// a `Config` before the bus exists), but prefer
    /// `enable_conversion_ready_pin*()` whenever hardware is available: a
    /// config-only change with stale threshold registers leaves RDY inactive.
    pub fn as_conversion_ready(&self) -> Self {
        Self { comp_que: CompQue::OneConversion, comp_lat: CompLat::NonLatching, ..*self }
    }
    /// True when the comparator field enables the ALERT/RDY output
    /// (`COMP_QUE != DisableComparator`). Every RDY event API requires this;
    /// with the struct default the pin is inactive and event waits hang.
    /// `enable_conversion_ready_pin*()` establishes this automatically.
    pub fn is_conversion_ready(&self) -> bool {
        !matches!(self.comp_que, CompQue::DisableComparator)
    }
}

/// Debug-only guard for the RDY precondition: panics in debug builds when an
/// event API is entered with a comparator-disabled config (which would hang).
#[inline]
fn debug_assert_rdy(config: &Config, what: &str) {
    debug_assert!(
        config.is_conversion_ready(),
        "{what}: COMP_QUE == DisableComparator; call enable_conversion_ready_pin*() or as_conversion_ready() first, else the RDY wait never completes"
    );
}

/// PGA[11:9]: Only for ADS1114/5.
#[derive(Debug, Clone, Copy, Default)]
#[repr(u8)]
pub enum Gain {
    #[default]
    V6_144 = 0,
    V4_096 = 1 << 1,
    V2_048 = 2 << 1,
    V1_024 = 3 << 1,
    V0_512 = 4 << 1,
    V0_256 = 5 << 1,
}

/// DR[7:5]: Data rate
#[derive(Debug, Clone, Copy, Default)]
#[repr(u8)]
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

/// COMP_MODE [4]: Only for ADS1114/5
#[derive(Debug, Clone, Copy, Default)]
#[repr(u8)]
pub enum CompMode {
    #[default]
    Traditional = 0,
    Window = 1 << 4,
}

/// COMP_POL [3]: Only for ADS1114/5
#[derive(Debug, Clone, Copy, Default)]
#[repr(u8)]
pub enum CompPol {
    #[default]
    ActiveLow = 0,
    ActiveHigh = 1 << 3,
}

/// COMP_LAT [2]: Only for ADS1114/5
#[derive(Debug, Clone, Copy, Default)]
#[repr(u8)]
pub enum CompLat {
    #[default]
    NonLatching = 0,
    Latching = 1 << 2,
}

/// COMP_QUE [1:0]: Only for ADS1114/5
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum CompQue {
    OneConversion = 0,
    TwoConversions = 1,
    FourConversions = 2,
    #[default]
    DisableComparator = 3,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum Address {
    #[default]
    Ground = 0b1001000,
    VDD = 0b1001001,
    SDA = 0b1001010,
    SCL = 0b1001011,
}

// ============================================================================
// Multi-device support: several ADS1115s on ONE I2C bus (bus-owning coordinator).
//
// ```text
//              ┌── ADS1115 @ 0x48 (Ground)
//              ├── ADS1115 @ 0x49 (VDD)
//  I2C BUS ────┼── ADS1115 @ 0x4A (SDA)
//              └── ADS1115 @ 0x4B (SCL)
// ```
// One bus + N addresses + per-device state. Max 4 directly-addressed devices
// per bus (ADDR pin); beyond that use an external I2C mux / second bus — the
// driver stays mux-agnostic (any `embedded-hal` I2C impl works).
//
// SEMANTICS (read carefully):
// - "Channels on one ADS1115" = multiplexed inputs on ONE converter (sequential).
// - "Multiple ADS1115s" = independent converters (parallel/overlapping).
// - 4 ADS1115 x 4 single-ended inputs = 16 logical channels (TI reference
//   design), but NOT 16 simultaneous samples: each ADS1115 still multiplexes.
// - Conversions started via sequential I2C writes give OVERLAPPING windows,
//   not phase-locked sampling. No hardware-sync claim is made.
// - Continuous mode has ONE conversion register per device (no FIFO):
//   `next_*` returns the LATEST sample; late readers lose intermediates.
// ============================================================================

/// Per-device configuration within a multi-ADC array.
#[derive(Debug, Clone, Copy)]
pub struct DeviceConfig {
    pub address: Address,
    pub config: Config,
    pub mux: Mux,
}

impl DeviceConfig {
    pub fn new(address: Address, config: Config, mux: Mux) -> Self {
        Self { address, config, mux }
    }
}

/// Single-device result retaining device identity (no alloc).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sample {
    pub index: usize,
    pub address: Address,
    pub value: i16,
}

/// Differential single-device result retaining identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiffSample {
    pub index: usize,
    pub address: Address,
    pub negative: bool,
    pub magnitude: u16,
}

/// Multi-device error: rejects duplicate addresses at construction; every
/// I2C/wait failure carries the device index + address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MultiError<I2cError, WaitError = ()> {
    DuplicateAddress { first: usize, second: usize, address: Address },
    InvalidIndex(usize),
    I2c { index: usize, address: Address, error: I2cError },
    Wait { index: usize, address: Address, error: WaitError },
}

/// Bus-owning coordinator for N ADS1115s sharing one I2C bus.
/// Owns the bus: exactly one transaction active at a time (Embassy-safe).
pub struct MultiADS1115<I2C, const N: usize> {
    i2c: I2C,
    devices: [DeviceConfig; N],
}

/// Four-device convenience alias (max directly-addressed ADS111x per bus).
pub type ADS1115Array<I2C> = MultiADS1115<I2C, 4>;

impl<I2C: embedded_hal::i2c::ErrorType, const N: usize> MultiADS1115<I2C, N> {
    /// Returns Err on duplicate addresses (use an external I2C mux if you must
    /// repeat an address behind isolated segments).
    pub fn new(i2c: I2C, devices: [DeviceConfig; N]) -> Result<Self, MultiError<I2cErrorOf<I2C>, ()>> {
        let _ = &i2c;
        let me = Self { i2c, devices };
        me.check_unique()?;
        Ok(me)
    }

    fn check_unique(&self) -> Result<(), MultiError<I2cErrorOf<I2C>, ()>> {
        let mut i = 0;
        while i < N {
            let mut j = i + 1;
            while j < N {
                if self.devices[i].address as u8 == self.devices[j].address as u8 {
                    return Err(MultiError::DuplicateAddress {
                        first: i, second: j, address: self.devices[i].address,
                    });
                }
                j += 1;
            }
            i += 1;
        }
        Ok(())
    }

    pub fn device(&self, index: usize) -> Option<&DeviceConfig> { self.devices.get(index) }
    pub fn device_mut(&mut self, index: usize) -> Option<&mut DeviceConfig> { self.devices.get_mut(index) }
    pub fn release(self) -> I2C { self.i2c }
}

/// Helper: I2C error type of a bus without requiring a full I2c bound at construction.
pub trait BusError {
    type Error;
}
impl<T: embedded_hal::i2c::ErrorType> BusError for T {
    type Error = T::Error;
}
type I2cErrorOf<B> = <B as BusError>::Error;

fn multi_write_config<I2C: I2c>(i2c: &mut I2C, addr: u8, b: [u8; 2]) -> Result<(), I2C::Error> {
    i2c.write(addr, &[REG_CONFIG, b[0], b[1]])
}
fn multi_read_conv<I2C: I2c>(i2c: &mut I2C, addr: u8) -> Result<i16, I2C::Error> {
    let mut d = [0u8; 2];
    i2c.write_read(addr, &[REG_CONVERSION], &mut d)?;
    Ok(i16::from_be_bytes(d))
}
fn multi_wait<I2C: I2c>(i2c: &mut I2C, addr: u8) -> Result<(), I2C::Error> {
    let mut buf = [0u8; 2];
    loop {
        i2c.write_read(addr, &[REG_CONFIG], &mut buf)?;
        if conversion_complete(buf[0]) { break; }
    }
    Ok(())
}

impl<I2C: I2c, const N: usize> MultiADS1115<I2C, N> {
    fn start_idx(&mut self, index: usize, mux: Mux) -> Result<(), MultiError<I2C::Error, ()>> {
        let dev = *self.devices.get(index).ok_or(MultiError::InvalidIndex(index))?;
        let (addr, cfg) = (dev.address, dev.config);
        let b = build_config_bytes(&cfg, mux as u8, true, Mode::SingleShot);
        multi_write_config(&mut self.i2c, addr as u8, b)
            .map_err(|e| MultiError::I2c { index, address: addr, error: e })
    }

    /// Start one device (single-shot, OS=1). Prefer `start_all` for overlap.
    pub fn start_conversion(&mut self, index: usize, mux: Mux) -> Result<(), MultiError<I2C::Error, ()>> {
        self.start_idx(index, mux)
    }

    /// Start ALL devices first (sequential I2C writes, overlapping conversions).
    ///
    /// # Failure semantics
    /// I2C writes are issued device by device, so an error partway through
    /// leaves earlier devices already running while later ones never started.
    /// The driver does NOT roll back: stopping the started devices would need
    /// further I2C transactions that could themselves fail, and silently
    /// reconfiguring hardware behind the caller's back is worse than reporting
    /// the error with its device index/address (`MultiError::I2c`). On error,
    /// re-establish a known state explicitly (e.g. re-issue the start for all
    /// devices, or reset/reconfigure the affected ones) before capturing.
    pub fn start_all(&mut self, muxes: [Mux; N]) -> Result<(), MultiError<I2C::Error, ()>> {
        let mut i = 0;
        while i < N {
            self.start_idx(i, muxes[i])?;
            i += 1;
        }
        Ok(())
    }

    fn read_idx(&mut self, index: usize) -> Result<Sample, MultiError<I2C::Error, ()>> {
        let addr = self.devices.get(index).map(|d| d.address).ok_or(MultiError::InvalidIndex(index))?;
        multi_wait(&mut self.i2c, addr as u8)
            .map_err(|e| MultiError::I2c { index, address: addr, error: e })?;
        let v = multi_read_conv(&mut self.i2c, addr as u8)
            .map_err(|e| MultiError::I2c { index, address: addr, error: e })?;
        Ok(Sample { index, address: addr, value: v })
    }

    /// Parallel capture with per-device MUX: START all, then WAIT+READ all.
    /// Returns samples in device order (index/address identity preserved).
    pub fn capture_parallel(&mut self, muxes: [Mux; N]) -> Result<[Sample; N], MultiError<I2C::Error, ()>>
    where [Sample; N]: Default {
        self.start_all(muxes)?;
        let mut out: [Sample; N] = Default::default();
        let mut i = 0;
        while i < N {
            out.as_mut()[i] = self.read_idx(i)?;
            i += 1;
        }
        Ok(out)
    }

    /// Capture using each device's stored `mux`.
    pub fn capture_all(&mut self) -> Result<[Sample; N], MultiError<I2C::Error, ()>>
    where [Sample; N]: Default {
        let mut muxes: [Mux; N] = [Mux::A0; N];
        let mut i = 0;
        while i < N {
            muxes[i] = self.devices[i].mux;
            i += 1;
        }
        self.capture_parallel(muxes)
    }

    /// Parallel differential capture -> [(negative, magnitude); N].
    pub fn capture_parallel_diff(&mut self, muxes: [Mux; N]) -> Result<[DiffSample; N], MultiError<I2C::Error, ()>>
    where [DiffSample; N]: Default {
        self.start_all(muxes)?;
        let mut out: [DiffSample; N] = Default::default();
        let mut i = 0;
        while i < N {
            let s = self.read_idx(i)?;
            let (negative, magnitude) = signed_to_polarity_magnitude(s.value);
            out.as_mut()[i] = DiffSample { index: i, address: s.address, negative, magnitude };
            i += 1;
        }
        Ok(out)
    }

    /// All devices enter continuous mode (MODE=0, OS=0), once each.
    ///
    /// # Failure semantics
    /// Same partial-start rule as `start_all`: an I2C error partway through
    /// leaves earlier devices converting while later ones never started, with
    /// no rollback. Re-establish a known state before streaming.
    pub fn start_continuous_all(&mut self, muxes: [Mux; N]) -> Result<(), MultiError<I2C::Error, ()>> {
        let mut i = 0;
        while i < N {
            let (addr, cfg) = (self.devices[i].address, self.devices[i].config);
            let b = build_config_bytes(&cfg, muxes[i] as u8, false, Mode::Continuous);
            multi_write_config(&mut self.i2c, addr as u8, b)
                .map_err(|e| MultiError::I2c { index: i, address: addr, error: e })?;
            i += 1;
        }
        Ok(())
    }

    // NOTE: there is deliberately NO blocking multi-RDY `next_all_blocking()`.
    // Sequential blocking waits cannot observe N independent ~8µs pulses: while
    // blocked on ADC0, ADC1..N can fire unseen. Use the concurrent async
    // `next_all_event()` or the timer-based `next_all_timed()` instead. (A
    // blocking multi API would additionally require level-latched ready flags
    // that stay asserted until serviced — hardware the driver cannot assume.)
    /// Latest sample per device via a caller-supplied delay: waits ~one
    /// (max) conversion period once, then reads all. No restart, no CONFIG
    /// polling, no RDY wiring needed. Timing is a conservative allowance, not
    /// synchronized timestamps (see timer docs).
    pub fn next_all_timed(
        &mut self,
        mut delay_us: impl FnMut(u32),
    ) -> Result<[Sample; N], MultiError<I2C::Error, ()>>
    where [Sample; N]: Default {
        let mut period = 0;
        let mut i = 0;
        while i < N {
            let p = conversion_period_us(self.devices[i].config.data_rate);
            if p > period {
                period = p;
            }
            i += 1;
        }
        delay_us(period);
        let mut out: [Sample; N] = Default::default();
        let mut i = 0;
        while i < N {
            let dev = self.devices[i];
            let v = multi_read_conv(&mut self.i2c, dev.address as u8)
                .map_err(|e| MultiError::I2c { index: i, address: dev.address, error: e })?;
            out.as_mut()[i] = Sample { index: i, address: dev.address, value: v };
            i += 1;
        }
        Ok(out)
    }

    // (no dummy-error helper: out-of-range indices use MultiError::InvalidIndex)
}

impl Default for Sample {
    fn default() -> Self { Self { index: 0, address: Address::Ground, value: 0 } }
}
impl Default for DiffSample {
    fn default() -> Self { Self { index: 0, address: Address::Ground, negative: false, magnitude: 0 } }
}

/// Bus-independent multi-device helpers: arming and concurrent waiting touch
/// only device configs and RDY sources, never the I2C bus, so they carry no
/// `AsyncI2c` bound and stay usable with any bus type.
#[cfg(feature = "async")]
impl<I2C: embedded_hal::i2c::ErrorType, const N: usize> MultiADS1115<I2C, N> {
    fn polarities(&self) -> [ConversionReadyPolarity; N] {
        core::array::from_fn(|i| {
            ConversionReadyPolarity::from(self.devices.get(i).map(|d| d.config.comp_pol).unwrap_or(CompPol::ActiveLow))
        })
    }
    /// Synchronously arm every RDY detector (per-device polarity from its
    /// config). Call this BEFORE starting any conversion: it is the only
    /// way to guarantee no ~8µs conversion-ready pulse fires while no
    /// detector is listening.
    ///
    /// # Precondition
    /// Every device must have conversion-ready mode enabled (threshold
    /// registers + `COMP_QUE != DisableComparator`); debug builds assert
    /// this per device, since an inactive RDY output would hang the wait.
    pub fn arm_all_ready<R: ConversionReady>(&self, rdy: &mut [R; N]) {
        let mut i = 0;
        while i < N {
            if let Some(dev) = self.devices.get(i) {
                debug_assert_rdy(&dev.config, "MultiADS1115::arm_all_ready");
            }
            let pol = ConversionReadyPolarity::from(
                self.devices.get(i).map(|d| d.config.comp_pol).unwrap_or(CompPol::ActiveLow),
            );
            rdy[i].arm(pol);
            i += 1;
        }
    }
    /// Wait for ALL devices concurrently (single join future; observes edges
    /// on detectors that must already have been armed via `arm_all_ready`
    /// before the conversions were started). Wait failures carry the failing
    /// device's index AND address.
    ///
    /// # Precondition
    /// Same conversion-ready precondition as `capture_parallel_event`
    /// (asserted per device in `arm_all_ready`; call it before starting).
    pub async fn wait_all_ready_async<R: ConversionReady>(
        &self,
        rdy: &mut [R; N],
    ) -> Result<(), MultiError<I2C::Error, R::Error>> {
        let mut i = 0;
        while i < N {
            if let Some(dev) = self.devices.get(i) {
                debug_assert_rdy(&dev.config, "MultiADS1115::wait_all_ready_async");
            }
            i += 1;
        }
        let pol = self.polarities();
        wait_all_ready(rdy, pol).await.map_err(|e| {
            let address = self
                .devices
                .get(e.index)
                .map(|d| d.address)
                .unwrap_or(Address::Ground);
            MultiError::Wait { index: e.index, address, error: e.error }
        })
    }
}

#[cfg(feature = "async")]
mod multi_async {
    use super::*;
    use embedded_hal_async::i2c::I2c as AsyncI2c;
    async fn mstart<I2C: AsyncI2c>(i2c: &mut I2C, addr: u8, cfg: &Config, mux: Mux) -> Result<(), I2C::Error> {
        let b = build_config_bytes(cfg, mux as u8, true, Mode::SingleShot);
        i2c.write(addr, &[REG_CONFIG, b[0], b[1]]).await
    }
    async fn mread<I2C: AsyncI2c>(i2c: &mut I2C, addr: u8) -> Result<i16, I2C::Error> {
        let mut d = [0u8; 2];
        i2c.write_read(addr, &[REG_CONVERSION], &mut d).await?;
        Ok(i16::from_be_bytes(d))
    }

    /// Lift a wait-free multi error (pure reads) into a caller expecting a wait
    /// error type. The `Wait` arm is unreachable by construction.
    fn lift_no_wait<I2cError, WaitError>(
        e: MultiError<I2cError, ()>,
    ) -> MultiError<I2cError, WaitError> {
        match e {
            MultiError::DuplicateAddress { first, second, address } => {
                MultiError::DuplicateAddress { first, second, address }
            }
            MultiError::InvalidIndex(i) => MultiError::InvalidIndex(i),
            MultiError::I2c { index, address, error } => MultiError::I2c { index, address, error },
            MultiError::Wait { .. } => unreachable!("pure read path never waits"),
        }
    }

    impl<I2C: AsyncI2c, const N: usize> MultiADS1115<I2C, N> {
        async fn start_idx_a(&mut self, index: usize, mux: Mux) -> Result<(), MultiError<I2C::Error, ()>> {
            let dev = *self.devices.get(index).ok_or(MultiError::InvalidIndex(index))?;
            mstart(&mut self.i2c, dev.address as u8, &dev.config, mux).await
                .map_err(|e| MultiError::I2c { index, address: dev.address, error: e })
        }
        /// START all (overlapping conversion windows; I2C writes serialized,
        /// conversions concurrent). Read with `read_started_all_event/_timer`.
        ///
        /// # Failure semantics
        /// Same partial-start rule as sync `start_all`: an I2C error partway
        /// through leaves earlier devices running with no rollback; the error
        /// carries the failing device index/address for recovery.
        pub async fn start_all_async(&mut self, muxes: [Mux; N]) -> Result<(), MultiError<I2C::Error, ()>> {
            let mut i = 0;
            while i < N {
                self.start_idx_a(i, muxes[i]).await?;
                i += 1;
            }
            Ok(())
        }
        async fn read_started_idx(
            &mut self,
            index: usize,
        ) -> Result<Sample, MultiError<I2C::Error, ()>> {
            let dev = *self.devices.get(index).ok_or(MultiError::InvalidIndex(index))?;
            let v = mread(&mut self.i2c, dev.address as u8).await
                .map_err(|e| MultiError::I2c { index, address: dev.address, error: e })?;
            Ok(Sample { index, address: dev.address, value: v })
        }
        /// Read conversion registers of already-started devices (no wait, no
        /// start, no RDY involved — hence no `ConversionReady` generic: callers
        /// need no fake RDY type just to read a completed conversion).
        pub async fn read_all_async(
            &mut self,
        ) -> Result<[Sample; N], MultiError<I2C::Error, ()>>
        where [Sample; N]: Default {
            let mut out: [Sample; N] = Default::default();
            let mut i = 0;
            while i < N {
                out.as_mut()[i] = self.read_started_idx(i).await?;
                i += 1;
            }
            Ok(out)
        }
        /// Parallel RDY capture: ARM all -> START all -> wait ALL concurrently
        /// -> READ all. Arming precedes every START, so no conversion-ready
        /// pulse can fire while its detector is unarmed. Each ADC needs its
        /// OWN RDY pin (`rdy[i]` <-> device i).
        ///
        /// # Precondition
        /// Every device must have conversion-ready mode enabled (threshold
        /// registers + `COMP_QUE != DisableComparator`, e.g. via
        /// `enable_conversion_ready_pin_async()` applied to each device's
        /// config). Otherwise RDY stays inactive and the wait hangs forever
        /// (debug builds assert this in `arm_all_ready`).
        ///
        /// # Failure semantics
        /// The START phase follows the partial-start rule (`start_all` docs).
        pub async fn capture_parallel_event<R: ConversionReady>(
            &mut self, muxes: [Mux; N], rdy: &mut [R; N],
        ) -> Result<[Sample; N], MultiError<I2C::Error, R::Error>>
        where [Sample; N]: Default {
            self.arm_all_ready(rdy);
            self.start_all_async(muxes).await.map_err(|e| match e {
                MultiError::InvalidIndex(i) => MultiError::InvalidIndex(i),
                MultiError::DuplicateAddress { first, second, address } => MultiError::DuplicateAddress { first, second, address },
                MultiError::I2c { index, address, error } => MultiError::I2c { index, address, error },
                MultiError::Wait { .. } => unreachable!(),
            })?;
            self.wait_all_ready_async(rdy).await?;
            self.read_all_async().await.map_err(lift_no_wait)
        }
        /// Parallel differential RDY capture (same start/wait/read pipeline).
        ///
        /// # Precondition
        /// Same conversion-ready precondition as `capture_parallel_event`.
        pub async fn capture_parallel_diff_event<R: ConversionReady>(
            &mut self, muxes: [Mux; N], rdy: &mut [R; N],
        ) -> Result<[DiffSample; N], MultiError<I2C::Error, R::Error>>
        where [DiffSample; N]: Default, [Sample; N]: Default {
            let samples = self.capture_parallel_event(muxes, rdy).await?;
            let mut out: [DiffSample; N] = Default::default();
            let mut i = 0;
            while i < N {
                let (negative, magnitude) = signed_to_polarity_magnitude(samples[i].value);
                out.as_mut()[i] = DiffSample { index: i, address: samples[i].address, negative, magnitude };
                i += 1;
            }
            Ok(out)
        }
        /// Multi-device continuous: MODE=0/OS=0 on every ADC, once each.
        /// UNCHECKED: call `arm_all_ready(rdy)` BEFORE this (the name is the
        /// warning) so no first pulse fires while unlistened
        /// (`next_all_event` re-arms every cycle, but arming before the
        /// initial START is what protects the first period).
        ///
        /// # Failure semantics
        /// Same partial-start rule as `start_all`: no rollback on I2C error.
        /// Prefer the combined `continuous_all_event()` constructor, which
        /// documents the full arm+start contract in one place.
        pub async fn start_continuous_all_async_unchecked(&mut self, muxes: [Mux; N]) -> Result<(), MultiError<I2C::Error, ()>> {
            let mut i = 0;
            while i < N {
                let dev = *self.devices.get(i).ok_or(MultiError::InvalidIndex(i))?;
                let b = build_config_bytes(&dev.config, muxes[i] as u8, false, Mode::Continuous);
                self.i2c.write(dev.address as u8, &[REG_CONFIG, b[0], b[1]]).await
                    .map_err(|e| MultiError::I2c { index: i, address: dev.address, error: e })?;
                i += 1;
            }
            Ok(())
        }
        /// Race-safe multi-device continuous session: arms ALL RDY detectors
        /// BEFORE starting every ADC (MODE=0/OS=0, once each). This is the
        /// event-driven counterpart of the single-ADC `continuous()`
        /// constructor — prefer it over manual `arm_all_ready()` +
        /// `start_continuous_all_async_unchecked()`. Each `next_all_event()` re-arms
        /// every cycle. Debug-asserts the RDY precondition per device.
        ///
        /// # Failure semantics
        /// The START phase follows the partial-start rule (`start_all` docs):
        /// an I2C error partway through leaves earlier devices running with no
        /// rollback; arming itself cannot fail.
        ///
        /// # Precondition
        /// Same conversion-ready precondition as `capture_parallel_event`.
        pub async fn continuous_all_event<R: ConversionReady>(
            &mut self,
            muxes: [Mux; N],
            rdy: &mut [R; N],
        ) -> Result<(), MultiError<I2C::Error, ()>> {
            self.arm_all_ready(rdy);
            self.start_continuous_all_async_unchecked(muxes).await
        }
        /// Latest sample per device: re-arm all, then concurrent wait-all and
        /// read all. No restart, no FIFO — late readers get the latest
        /// conversion only. Re-arming each cycle means a pulse in flight
        /// between cycles resolves to the NEXT period's pulse (a sample may be
        /// skipped, but the wait cannot hang on an already-consumed edge).
        ///
        /// # Precondition
        /// Same conversion-ready precondition as `capture_parallel_event`
        /// (asserted per device on every re-arm).
        pub async fn next_all_event<R: ConversionReady>(
            &mut self, rdy: &mut [R; N],
        ) -> Result<[Sample; N], MultiError<I2C::Error, R::Error>>
        where [Sample; N]: Default {
            self.arm_all_ready(rdy);
            self.wait_all_ready_async(rdy).await?;
            self.read_all_async().await.map_err(lift_no_wait)
        }
    }

    // ---- Embassy Timer multi-device capture (feature "embassy") ----
    #[cfg(feature = "embassy")]
    impl<I2C: AsyncI2c, const N: usize> MultiADS1115<I2C, N> {
        fn max_period_us(&self) -> u32 {
            let mut m = 0;
            let mut i = 0;
            while i < N {
                let p = conversion_period_us(self.devices[i].config.data_rate);
                if p > m { m = p; }
                i += 1;
            }
            m
        }
        /// Parallel timer capture: START all, ONE shared `Timer::after(max period)`,
        /// then READ all. Conversions overlap; CONFIG never rewritten per sample.
        /// Allows every device a conservative conversion window; it does
        /// NOT guarantee synchronized sample timestamps (faster ADCs may have
        /// completed several conversions — reads return the LATEST sample).
        pub async fn capture_parallel_timer(&mut self, muxes: [Mux; N]) -> Result<[Sample; N], MultiError<I2C::Error, ()>>
        where [Sample; N]: Default {
            self.start_all_async(muxes).await?;
            let period = self.max_period_us();
            embassy_time::Timer::after(embassy_time::Duration::from_micros(period as u64)).await;
            let mut out: [Sample; N] = Default::default();
            let mut i = 0;
            while i < N {
                let dev = self.devices[i];
                let v = mread(&mut self.i2c, dev.address as u8).await
                    .map_err(|e| MultiError::I2c { index: i, address: dev.address, error: e })?;
                out.as_mut()[i] = Sample { index: i, address: dev.address, value: v };
                i += 1;
            }
            Ok(out)
        }
        pub async fn capture_all_timer(&mut self) -> Result<[Sample; N], MultiError<I2C::Error, ()>>
        where [Sample; N]: Default {
            let mut muxes: [Mux; N] = [Mux::A0; N];
            let mut i = 0;
            while i < N {
                muxes[i] = self.devices[i].mux;
                i += 1;
            }
            self.capture_parallel_timer(muxes).await
        }
        pub async fn capture_parallel_diff_timer(&mut self, muxes: [Mux; N]) -> Result<[DiffSample; N], MultiError<I2C::Error, ()>>
        where [DiffSample; N]: Default {
            self.start_all_async(muxes).await?;
            let period = self.max_period_us();
            embassy_time::Timer::after(embassy_time::Duration::from_micros(period as u64)).await;
            let mut out: [DiffSample; N] = Default::default();
            let mut i = 0;
            while i < N {
                let dev = self.devices[i];
                let v = mread(&mut self.i2c, dev.address as u8).await
                    .map_err(|e| MultiError::I2c { index: i, address: dev.address, error: e })?;
                let (negative, magnitude) = signed_to_polarity_magnitude(v);
                out.as_mut()[i] = DiffSample { index: i, address: dev.address, negative, magnitude };
                i += 1;
            }
            Ok(out)
        }
        /// Timer continuous: start all once, then per `next_all_timer`:
        /// Timer::after(max period) + read all (no restart).
        pub async fn start_continuous_all_timer(&mut self, muxes: [Mux; N]) -> Result<(), MultiError<I2C::Error, ()>> {
            self.start_continuous_all_async_unchecked(muxes).await
        }
        pub async fn next_all_timer(&mut self) -> Result<[Sample; N], MultiError<I2C::Error, ()>>
        where [Sample; N]: Default {
            let period = self.max_period_us();
            embassy_time::Timer::after(embassy_time::Duration::from_micros(period as u64)).await;
            let mut out: [Sample; N] = Default::default();
            let mut i = 0;
            while i < N {
                let dev = self.devices[i];
                let v = mread(&mut self.i2c, dev.address as u8).await
                    .map_err(|e| MultiError::I2c { index: i, address: dev.address, error: e })?;
                out.as_mut()[i] = Sample { index: i, address: dev.address, value: v };
                i += 1;
            }
            Ok(out)
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    #[test]
    fn config_bytes_mux_gain_dr() {
        let cfg = Config { gain: Gain::V2_048, data_rate: DataRate::SPS128, ..Default::default() };
        // MUX A0=100, gain V2_048=2<<1=4, OS=1, MODE single=1
        let b = build_config_bytes(&cfg, Mux::A0 as u8, true, Mode::SingleShot);
        assert_eq!(b[0], 0x80 | (0b100 << 4) | 0x04 | 0x01);
        assert_eq!(b[1], (4 << 5) | 3); // DR + COMP_QUE disabled default
        assert!(conversion_complete(0x80) && !conversion_complete(0x00));
    }

    #[test]
    fn all_mux_bits() {
        for (m, bits) in [(Mux::A0N1,0),(Mux::A0N3,1),(Mux::A1N3,2),(Mux::A2N3,3),(Mux::A0,4),(Mux::A1,5),(Mux::A2,6),(Mux::A3,7)] {
            let b = build_config_bytes(&Config::default(), m as u8, true, Mode::SingleShot);
            assert_eq!((b[0] >> 4) & 7, bits);
        }
    }

    #[test]
    fn ads1113_ignores_pga() {
        let b = build_config_bytes_ads1113(DataRate::SPS860, true);
        assert_eq!(b[0] & 0x0E, 0); // no PGA bits
        assert_eq!(b[1] & 3, 3); // comparator disabled
    }

    #[test]
    fn continuous_clears_mode_bit() {
        let b = build_config_bytes(&Config::default(), Mux::A0 as u8, false, Mode::Continuous);
        assert_eq!(b[0] & 1, 0);
        assert_eq!(b[0] & 0x80, 0, "continuous must not set OS=1");
        let s = build_config_bytes(&Config::default(), Mux::A0 as u8, true, Mode::SingleShot);
        assert_eq!(s[0] & 1, 1);
        assert_ne!(s[0] & 0x80, 0);
        // ADS1113 continuous: MODE=0 (bit0=0), OS=0.
        #[cfg(feature = "embassy")]
        {
            let c = build_config_bytes_ads1113_continuous(DataRate::SPS128);
            assert_eq!(c, [0, DataRate::SPS128 as u8 | 0x03]);
        }
    }

    #[cfg(feature = "embassy")]
    #[test]
    fn embassy_timer_api_no_poll() {
        // Timer paths: single CONFIG write per start, Timer::after for sync,
        // never a CONFIG read loop. `next()` = wait + read (no restart).
        let src = include_str!("lib.rs");
        for name in ["fn read_channel_timer", "fn continuous_timer", "fn read_a0_a1_timer",
            "fn read_adc_a0_timer", "Timer::after", "ContinuousTimer"] {
            assert!(src.contains(name), "missing {name}");
        }
        let m = src.find("mod async_api").expect("async_api");
        let end = src[m..].find("/// Embassy integration").map(|e| m + e).unwrap_or(src.len());
        assert!(!src[m..end].contains("read_asyncadc"));
    }

    #[test]
    fn rdy_threshold_values() {
        // Conversion-ready requires Hi=0x8000, Lo=0x0000 and COMP_QUE != 11b.
        let cfg = Config::default().as_conversion_ready();
        assert_ne!(cfg.comp_que as u8, 3);
        assert_eq!([0x80u8, 0x00u8], [0x80, 0x00]); // Hi_thresh bytes
        assert_eq!([0x00u8, 0x00u8], [0x00, 0x00]); // Lo_thresh bytes
    }

    #[test]
    fn polarity_matches_comp_pol() {
        assert_eq!(ConversionReadyPolarity::from(CompPol::ActiveLow), ConversionReadyPolarity::ActiveLow);
        assert_eq!(ConversionReadyPolarity::from(CompPol::ActiveHigh), ConversionReadyPolarity::ActiveHigh);
        assert!(conversion_period_us(DataRate::SPS8) > conversion_period_us(DataRate::SPS860));
    }

    #[test]
    fn polarity_magnitude() {
        assert_eq!(signed_to_polarity_magnitude(0), (false, 0));
        assert_eq!(signed_to_polarity_magnitude(123), (false, 123));
        assert_eq!(signed_to_polarity_magnitude(-123), (true, 123));
        assert_eq!(signed_to_polarity_magnitude(i16::MAX), (false, 32767));
        assert_eq!(signed_to_polarity_magnitude(i16::MIN), (true, 32768));
    }

    #[test]
    fn diff_mux_mappings() {
        assert_eq!(Mux::A0N1 as u8, 0b000);
        assert_eq!(Mux::A0N3 as u8, 0b001);
        assert_eq!(Mux::A1N3 as u8, 0b010);
        assert_eq!(Mux::A2N3 as u8, 0b011);
        for (m, bits) in [(Mux::A0N1, 0), (Mux::A0N3, 1), (Mux::A1N3, 2), (Mux::A2N3, 3)] {
            let b = build_config_bytes(&Config::default(), m as u8, true, Mode::SingleShot);
            assert_eq!((b[0] >> 4) & 7, bits);
        }
        // as_conversion_ready forces COMP_QUE != 11b, non-latching.
        let c = Config::default().as_conversion_ready();
        assert_ne!(c.comp_que as u8, 3);
        assert_eq!(c.comp_lat as u8, 0);
    }

    #[cfg(feature = "async")]
    #[test]
    fn async_recommended_api_has_no_poll_loop() {        // Regression: async module must contain no `loop` polling at all
        // (legacy polling removed); expected flow is write CONFIG -> await RDY -> read CONVERSION.
        let src = include_str!("lib.rs");
        let m = src.find("mod async_api").expect("async_api");
        let end = src[m..].find("mod tests").map(|e| m + e).unwrap_or(src.len());
        let body = &src[m..end];
        // Async modules must contain no `loop {` CONFIG polling. (Sync
        // `multi_wait`/`wait_complete_sync` legitimately poll — sync has no
        // event source — so check per async fn instead of the whole slice.)
        for name in ["fn read_channel_event", "fn read_channel_timed", "fn next_conversion", "fn continuous",
            "fn start_all_async", "fn capture_parallel_event", "fn capture_parallel_timer", "fn next_all_timer",
            "fn wait_all_ready_async", "fn read_all_async", "fn next_all_event"] {
            let i = body.find(name).unwrap_or_else(|| panic!("missing {name}"));
            let fn_body = &body[i..body[i..].find("/// ").map(|e| i + e).unwrap_or(body.len())];
            let fn_body = &fn_body[..fn_body.find("\n    }\n").unwrap_or(fn_body.len())];
            assert!(!fn_body.contains("loop {"), "{name} must not poll");
        }
        assert!(!body.contains("read_asyncadc") && !body.contains("read_async_adc") && !body.contains("read_async_4adc"));
        // Structural guard for the critical race fix: parallel capture must go
        // through the concurrent join, never sequential per-device awaits.
        // (JoinAllReady lives at crate root, ahead of `mod async_api`.)
        for must in ["wait_all_ready(", "JoinAllReady", "read_all_async"] {
            assert!(src.contains(must), "missing concurrent primitive {must}");
        }
    }

    // ---- concurrent RDY test: pulse dies unless EVERY detector is armed upfront ----
    #[cfg(feature = "async")]
    struct PulseRdy<'c> {
        clock: &'c std::cell::Cell<u32>,
        log: &'c std::cell::RefCell<std::vec::Vec<(u8, usize)>>,
        id: usize,
        armed_at: std::cell::Cell<Option<u32>>,
        arms: std::cell::Cell<u32>,
        polls: std::cell::Cell<u32>,
        fire_after: u32,
        deadline: u32,
    }
    #[cfg(feature = "async")]
    struct PulseFut<'a, 'c> {
        rdy: &'a PulseRdy<'c>,
    }
    #[cfg(feature = "async")]
    impl<'c> ConversionReady for PulseRdy<'c> {
        type Error = core::convert::Infallible;
        type WaitFuture<'a> = PulseFut<'a, 'c> where Self: 'a;
        fn arm(&mut self, _pol: ConversionReadyPolarity) {
            // Synchronous hardware arming: records the clock instant from which
            // the (short) pulse is observable. Tag 0 = arm event in order log.
            // Every call re-arms (mirrors re-enabling the edge detector each
            // continuous cycle); the pulse is observable iff the LATEST arm
            // precedes the conversion completion (deadline).
            self.armed_at.set(Some(self.clock.get()));
            self.arms.set(self.arms.get() + 1);
            self.log.borrow_mut().push((0, self.id));
        }
        fn wait_for_ready(&mut self, _pol: ConversionReadyPolarity) -> Self::WaitFuture<'_> {
            // Tag 2 = wait intent (future creation) in order log.
            self.log.borrow_mut().push((2, self.id));
            PulseFut { rdy: self }
        }
    }
    #[cfg(feature = "async")]
    impl core::future::Future for PulseFut<'_, '_> {
        type Output = Result<(), core::convert::Infallible>;
        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> core::task::Poll<Self::Output> {
            let r = self.rdy;
            // Unarmed detector observes nothing (mirrors hardware: no interrupt
            // registered => pulse passes unseen), even once polled.
            let armed = match r.armed_at.get() {
                Some(t) => t,
                None => return core::task::Poll::Pending,
            };
            // Pulse already gone if the detector was armed too late.
            if armed > r.deadline {
                return core::task::Poll::Pending; // never fires: missed ~8µs pulse
            }
            let p = r.polls.get() + 1;
            r.polls.set(p);
            if p >= r.fire_after {
                core::task::Poll::Ready(Ok(()))
            } else {
                cx.waker().wake_by_ref();
                core::task::Poll::Pending
            }
        }
    }

    #[cfg(feature = "async")]
    #[test]
    fn wait_all_ready_arms_every_detector_upfront() {
        use std::sync::Arc;
        struct Noop;
        impl std::task::Wake for Noop {
            fn wake(self: Arc<Self>) {}
        }
        let waker = std::task::Waker::from(Arc::new(Noop));
        let clock = std::cell::Cell::new(0u32);
        let log = std::cell::RefCell::new(std::vec::Vec::new());
        // Pulse observable ONLY if armed at clock 0 with 2 self-polls to fire:
        // a sequential wait(RDY0).await; wait(RDY1).await; ... implementation
        // would arm detectors 1..3 late and hang forever here. Per the trait
        // contract, arming is the driver's explicit synchronous step BEFORE
        // polling: mirror `capture_parallel_event` (arm all, then join).
        let mut rdy: [PulseRdy; 4] = core::array::from_fn(|id| PulseRdy {
            clock: &clock,
            log: &log,
            id,
            armed_at: std::cell::Cell::new(None),
            arms: std::cell::Cell::new(0),
            polls: std::cell::Cell::new(0),
            fire_after: 2,
            deadline: 0,
        });
        for r in rdy.iter_mut() {
            r.arm(ConversionReadyPolarity::ActiveLow);
        }
        assert_eq!(log.borrow().len(), 4, "all detectors armed before first poll");
        let mut join = wait_all_ready(&mut rdy, [ConversionReadyPolarity::ActiveLow; 4]);
        let mut join = unsafe { core::pin::Pin::new_unchecked(&mut join) };
        let mut cx = core::task::Context::from_waker(&waker);
        let mut rounds = 0;
        let done = loop {
            clock.set(rounds);
            match join.as_mut().poll(&mut cx) {
                core::task::Poll::Ready(r) => break r,
                core::task::Poll::Pending => {}
            }
            rounds += 1;
            assert!(rounds < 100, "RDY pulse missed: detectors not observed concurrently");
        };
        assert!(done.is_ok());
        for r in rdy.iter() {
            assert_eq!(r.armed_at.get(), Some(0), "every detector armed before START");
        }
    }

    #[cfg(feature = "async")]
    #[test]
    fn unarmed_join_observes_nothing() {
        // Contract enforcement: without the explicit synchronous arm step, the
        // join must NOT complete — this is what makes arm-before-start load
        // bearing rather than advisory.
        use std::sync::Arc;
        struct Noop;
        impl std::task::Wake for Noop {
            fn wake(self: Arc<Self>) {}
        }
        let waker = std::task::Waker::from(Arc::new(Noop));
        let clock = std::cell::Cell::new(0u32);
        let log = std::cell::RefCell::new(std::vec::Vec::new());
        let mut rdy: [PulseRdy; 2] = core::array::from_fn(|id| PulseRdy {
            clock: &clock,
            log: &log,
            id,
            armed_at: std::cell::Cell::new(None),
            arms: std::cell::Cell::new(0),
            polls: std::cell::Cell::new(0),
            fire_after: 1,
            deadline: u32::MAX,
        });
        let mut join = wait_all_ready(&mut rdy, [ConversionReadyPolarity::ActiveLow; 2]);
        let mut join = unsafe { core::pin::Pin::new_unchecked(&mut join) };
        let mut cx = core::task::Context::from_waker(&waker);
        let mut ready = false;
        for _ in 0..20 {
            if matches!(join.as_mut().poll(&mut cx), core::task::Poll::Ready(_)) {
                ready = true;
                break;
            }
        }
        assert!(!ready, "un-armed detectors must observe nothing");
    }

    // ---- wait-error attribution: failing device index + address, not index 0 ----
    #[cfg(feature = "async")]
    struct FailRdy {
        fail: bool,
        armed: std::cell::Cell<bool>,
    }
    #[cfg(feature = "async")]
    struct FailFut<'a> {
        rdy: &'a FailRdy,
    }
    #[cfg(feature = "async")]
    impl ConversionReady for FailRdy {
        type Error = MockErr;
        type WaitFuture<'a> = FailFut<'a> where Self: 'a;
        fn arm(&mut self, _pol: ConversionReadyPolarity) {
            self.armed.set(true);
        }
        fn wait_for_ready(&mut self, _pol: ConversionReadyPolarity) -> Self::WaitFuture<'_> {
            FailFut { rdy: self }
        }
    }
    #[cfg(feature = "async")]
    impl core::future::Future for FailFut<'_> {
        type Output = Result<(), MockErr>;
        fn poll(
            self: core::pin::Pin<&mut Self>,
            cx: &mut core::task::Context<'_>,
        ) -> core::task::Poll<Self::Output> {
            if self.rdy.fail {
                return core::task::Poll::Ready(Err(MockErr));
            }
            if self.rdy.armed.get() {
                core::task::Poll::Ready(Ok(()))
            } else {
                cx.waker().wake_by_ref();
                core::task::Poll::Pending
            }
        }
    }

    #[cfg(feature = "async")]
    #[test]
    fn wait_failure_attributes_correct_device() {
        // Device 2 (address SDA) fails: join reports index 2, and the multi
        // wrapper maps it to index 2 + address SDA (previously hardcoded 0).
        let mut rdy = [
            FailRdy { fail: false, armed: std::cell::Cell::new(false) },
            FailRdy { fail: false, armed: std::cell::Cell::new(false) },
            FailRdy { fail: true, armed: std::cell::Cell::new(false) },
            FailRdy { fail: false, armed: std::cell::Cell::new(false) },
        ];
        for r in rdy.iter_mut() {
            r.arm(ConversionReadyPolarity::ActiveLow);
        }
        let err = block_on(wait_all_ready(&mut rdy, [ConversionReadyPolarity::ActiveLow; 4]));
        match err {
            Err(WaitFailed { index: 2, .. }) => {}
            other => panic!("expected failure at index 2, got {other:?}"),
        }

        let cfg = Config::default().as_conversion_ready();
        let multi = MultiADS1115::new(
            MockI2c::new(),
            [
                DeviceConfig::new(Address::Ground, cfg, Mux::A0),
                DeviceConfig::new(Address::VDD, cfg, Mux::A1),
                DeviceConfig::new(Address::SDA, cfg, Mux::A2),
                DeviceConfig::new(Address::SCL, cfg, Mux::A3),
            ],
        )
        .unwrap();
        let mut rdy = [
            FailRdy { fail: false, armed: std::cell::Cell::new(false) },
            FailRdy { fail: false, armed: std::cell::Cell::new(false) },
            FailRdy { fail: true, armed: std::cell::Cell::new(false) },
            FailRdy { fail: false, armed: std::cell::Cell::new(false) },
        ];
        let err = block_on(multi.wait_all_ready_async(&mut rdy));
        match err {
            Err(MultiError::Wait { index: 2, address: Address::SDA, .. }) => {}
            other => panic!("expected Wait{{index 2, SDA}}, got {other:?}"),
        }
    }

    // ---- end-to-end: capture_parallel_event arms ALL before ANY start ----
    #[cfg(feature = "async")]
    struct AsyncMockI2c<'c> {
        log: &'c std::cell::RefCell<std::vec::Vec<(u8, usize)>>,
        canned: [(u8, i16); 2],
    }
    #[cfg(feature = "async")]
    impl embedded_hal::i2c::ErrorType for AsyncMockI2c<'_> {
        type Error = MockErr;
    }
    #[cfg(feature = "async")]
    impl embedded_hal_async::i2c::I2c for AsyncMockI2c<'_> {
        async fn read(&mut self, _a: u8, _b: &mut [u8]) -> Result<(), Self::Error> { Ok(()) }
        async fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            if !bytes.is_empty() && bytes[0] == REG_CONFIG {
                // Tag 1 = CONFIG START write; records detector-arm coverage implicitly
                // via shared order log (tag 0 = arm from PulseRdy).
                self.log.borrow_mut().push((1, (addr - 0x48) as usize));
            }
            Ok(())
        }
        async fn write_read(&mut self, addr: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
            if write == [REG_CONVERSION] {
                let v = self.canned.iter().find(|(a, _)| *a == addr).map(|(_, v)| *v).unwrap_or(0);
                read.copy_from_slice(&v.to_be_bytes()[..read.len().min(2)]);
            }
            Ok(())
        }
        async fn transaction(
            &mut self,
            _a: u8,
            _o: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[cfg(feature = "async")]
    fn block_on<F: core::future::Future>(mut f: F) -> F::Output {
        use std::sync::Arc;
        struct Noop;
        impl std::task::Wake for Noop {
            fn wake(self: Arc<Self>) {}
        }
        let waker = std::task::Waker::from(Arc::new(Noop));
        let mut cx = core::task::Context::from_waker(&waker);
        // SAFETY: future never moved after pinning; driven to completion here.
        let mut f = unsafe { core::pin::Pin::new_unchecked(&mut f) };
        let mut rounds = 0u32;
        loop {
            match f.as_mut().poll(&mut cx) {
                core::task::Poll::Ready(v) => return v,
                core::task::Poll::Pending => {}
            }
            rounds += 1;
            assert!(rounds < 1000, "async test future stalled");
        }
    }

    #[cfg(feature = "async")]
    #[test]
    fn capture_parallel_event_arms_before_any_start() {
        let clock = std::cell::Cell::new(0u32);
        let log = std::cell::RefCell::new(std::vec::Vec::new());
        let i2c = AsyncMockI2c { log: &log, canned: [(0x48, 1000), (0x49, -2000)] };
        let cfg = Config::default().with_data_rate(DataRate::SPS860).as_conversion_ready();
        let mut multi = MultiADS1115::new(
            i2c,
            [DeviceConfig::new(Address::Ground, cfg, Mux::A0), DeviceConfig::new(Address::VDD, cfg, Mux::A1)],
        )
        .unwrap();
        let mut rdy: [PulseRdy; 2] = core::array::from_fn(|id| PulseRdy {
            clock: &clock,
            log: &log,
            id,
            armed_at: std::cell::Cell::new(None),
            arms: std::cell::Cell::new(0),
            polls: std::cell::Cell::new(0),
            fire_after: 1,
            deadline: u32::MAX,
        });
        let samples = block_on(multi.capture_parallel_event([Mux::A0, Mux::A1], &mut rdy)).unwrap();
        assert_eq!([samples[0].value, samples[1].value], [1000, -2000]);
        assert_eq!([samples[0].address, samples[1].address], [Address::Ground, Address::VDD]);
        // Order proof: every arm (tag 0) precedes every CONFIG START (tag 1).
        let log = log.borrow();
        let first_start = log.iter().position(|(t, _)| *t == 1).expect("starts logged");
        assert!(log.iter().take(first_start).all(|(t, _)| *t == 0));
        assert_eq!(log.iter().filter(|(t, _)| *t == 0).count(), 2, "both detectors armed");
        assert_eq!(log.iter().filter(|(t, _)| *t == 1).count(), 2, "both ADCs started");
    }

    #[cfg(feature = "async")]
    #[test]
    fn continuous_all_event_arms_before_any_start() {
        // The combined continuous constructor must arm ALL detectors before
        // the first CONFIG START (MODE=0/OS=0), then next_all_event() must
        // re-arm every cycle before waiting.
        let clock = std::cell::Cell::new(0u32);
        let log = std::cell::RefCell::new(std::vec::Vec::new());
        let i2c = AsyncMockI2c { log: &log, canned: [(0x48, 1000), (0x49, -2000)] };
        let cfg = Config::default().with_data_rate(DataRate::SPS860).as_conversion_ready();
        let mut multi = MultiADS1115::new(
            i2c,
            [DeviceConfig::new(Address::Ground, cfg, Mux::A0), DeviceConfig::new(Address::VDD, cfg, Mux::A1)],
        )
        .unwrap();
        let mut rdy: [PulseRdy; 2] = core::array::from_fn(|id| PulseRdy {
            clock: &clock,
            log: &log,
            id,
            armed_at: std::cell::Cell::new(None),
            arms: std::cell::Cell::new(0),
            polls: std::cell::Cell::new(0),
            fire_after: 1,
            deadline: u32::MAX,
        });
        block_on(multi.continuous_all_event([Mux::A0, Mux::A1], &mut rdy)).unwrap();
        // Order proof: every arm (tag 0) precedes every CONFIG START (tag 1).
        // MODE=0/OS=0 continuous starts: exactly 2 writes, both non-OS, MODE clear.
        {
            let log = log.borrow();
            let first_start = log.iter().position(|(t, _)| *t == 1).expect("starts logged");
            assert!(log.iter().take(first_start).all(|(t, _)| *t == 0));
            assert_eq!(log.iter().filter(|(t, _)| *t == 0).count(), 2, "both detectors armed");
            assert_eq!(log.iter().filter(|(t, _)| *t == 1).count(), 2, "both ADCs started once");
        }
        // One full cycle: re-arm all (tag 0), wait intents (tag 2), then reads.
        let samples = block_on(multi.next_all_event(&mut rdy)).unwrap();
        assert_eq!([samples[0].value, samples[1].value], [1000, -2000]);
        let log = log.borrow();
        let tags: std::vec::Vec<u8> = log.iter().map(|(t, _)| *t).collect();
        assert_eq!(&tags[..6], &[0, 0, 1, 1, 0, 0], "arm,arm,start,start then re-arm,re-arm");
        assert_eq!(tags.iter().filter(|t| **t == 2).count(), 2, "both waits observed");
    }

    #[cfg(feature = "async")]
    #[test]
    fn single_shot_event_arms_before_start() {
        // The single-ADC race from review: read_channel_event must arm RDY
        // BEFORE writing CONFIG START, not after.
        let clock = std::cell::Cell::new(0u32);
        let log = std::cell::RefCell::new(std::vec::Vec::new());
        let i2c = AsyncMockI2c { log: &log, canned: [(0x48, 555), (0x49, 0)] };
        let mut adc = ADS1115::new(Address::Ground, i2c, Config::default().with_data_rate(DataRate::SPS860).as_conversion_ready());
        let mut rdy = PulseRdy {
            clock: &clock,
            log: &log,
            id: 0,
            armed_at: std::cell::Cell::new(None),
            arms: std::cell::Cell::new(0),
            polls: std::cell::Cell::new(0),
            fire_after: 1,
            deadline: u32::MAX,
        };
        let v = block_on(adc.read_channel_event(Mux::A0, &mut rdy)).unwrap();
        assert_eq!(v, 555);
        let log = log.borrow();
        // arm(0) -> CONFIG START(1) -> wait intent(2): arm precedes START.
        assert_eq!(log.as_slice(), &[(0, 0), (1, 0), (2, 0)], "arm must precede CONFIG START");
    }

    #[cfg(feature = "async")]
    #[test]
    fn continuous_next_rearms_every_cycle() {
        // Regression for the single-ADC continuous re-arm race: each
        // `next()` must be arm -> wait -> read, so a ~8µs pulse firing between
        // cycles is observed rather than missed. Proves the exact event order
        // across stream construction + two samples, for ADS1115 and ADS1114.
        let clock = std::cell::Cell::new(0u32);
        let log = std::cell::RefCell::new(std::vec::Vec::new());
        // ADS1115 stream: arm, START, then per-sample arm -> wait -> read.
        let i2c = AsyncMockI2c { log: &log, canned: [(0x48, 111), (0x49, 0)] };
        let mut adc = ADS1115::new(Address::Ground, i2c, Config::default().as_conversion_ready());
        let mut rdy = PulseRdy {
            clock: &clock,
            log: &log,
            id: 0,
            armed_at: std::cell::Cell::new(None),
            arms: std::cell::Cell::new(0),
            polls: std::cell::Cell::new(0),
            fire_after: 1,
            deadline: u32::MAX,
        };
        let mut stream = block_on(adc.continuous(Mux::A0, &mut rdy)).unwrap();
        assert_eq!(block_on(stream.next()).unwrap(), 111);
        assert_eq!(block_on(stream.next()).unwrap(), 111);
        drop(stream);
        assert_eq!(
            log.borrow().as_slice(),
            &[(0, 0), (1, 0), (0, 0), (2, 0), (0, 0), (2, 0)],
            "continuous() arms before START; every next() re-arms before waiting"
        );
        assert_eq!(rdy.arms.get(), 3, "constructor + one re-arm per next()");
        // ADS1114 stream constructor shares the same guarantee (no MUX arg).
        log.borrow_mut().clear();
        let i2c = AsyncMockI2c { log: &log, canned: [(0x48, 222), (0x49, 0)] };
        let mut adc = ADS1114::new(i2c, Config::default().as_conversion_ready());
        let mut rdy = PulseRdy {
            clock: &clock,
            log: &log,
            id: 0,
            armed_at: std::cell::Cell::new(None),
            arms: std::cell::Cell::new(0),
            polls: std::cell::Cell::new(0),
            fire_after: 1,
            deadline: u32::MAX,
        };
        let mut stream = block_on(adc.continuous(&mut rdy)).unwrap();
        assert_eq!(block_on(stream.next()).unwrap(), 222);
        drop(stream);
        assert_eq!(
            log.borrow().as_slice(),
            &[(0, 0), (1, 0), (0, 0), (2, 0)],
            "ADS1114 continuous() arms before START; next() re-arms"
        );
        assert_eq!(rdy.arms.get(), 2);
    }

    #[test]
    fn enable_rdy_upgrades_comparator_config() {
        // Option A: enabling RDY must leave the device actually usable, i.e.
        // COMP_QUE != DisableComparator despite the struct default.
        let mut adc = ADS1115::new(Address::Ground, MockI2c::new(), Config::default());
        assert_eq!(adc.config.comp_que as u8, CompQue::DisableComparator as u8);
        adc.enable_conversion_ready_pin().unwrap();
        assert_ne!(adc.config.comp_que as u8, CompQue::DisableComparator as u8);
        assert_eq!(adc.config.comp_lat as u8, CompLat::NonLatching as u8);
        // Two threshold writes: HI=0x8000, LO=0x0000.
        let thresh: std::vec::Vec<(u8, std::vec::Vec<u8>)> =
            adc.i2c.writes.iter().cloned().collect();
        assert_eq!(thresh.len(), 2);
        assert_eq!(thresh[0].1.as_slice(), &[REG_HI_THRESH, 0x80, 0x00]);
        assert_eq!(thresh[1].1.as_slice(), &[REG_LO_THRESH, 0x00, 0x00]);
    }

    #[test]
    fn rdy_precondition_predicate_and_guard() {
        // Default config leaves RDY inactive; the predicate reports it and the
        // event entry points refuse it in debug builds instead of hanging.
        assert!(!Config::default().is_conversion_ready());
        assert!(Config::default().as_conversion_ready().is_conversion_ready());
    }

    #[test]
    #[should_panic(expected = "COMP_QUE == DisableComparator")]
    fn event_api_rejects_disabled_comparator_in_debug() {
        // Entering an RDY event API without conversion-ready mode must fail
        // loudly in debug builds rather than hang on an inactive pin.
        let mut adc = ADS1115::new(Address::Ground, MockI2c::new(), Config::default());
        adc.start_continuous(Mux::A0).unwrap();
        let mut rdy = MockBlockingRdy { waited: std::cell::Cell::new(0), armed: std::cell::Cell::new(0) };
        let _ = adc.next_conversion_blocking(&mut rdy);
    }

    #[test]
    fn timer_margin_is_proportional_not_blanket() {
        // ~12.5% + 100µs: full-rate friendly, still strictly above nominal.
        // 860 SPS: nominal 1162µs -> ~1407µs (≈710 reads/s, was ≈462 with +1ms).
        assert_eq!(conversion_period_us(DataRate::SPS860), 1162 + 145 + 100);
        assert_eq!(conversion_period_us(DataRate::SPS8), 125_000 + 15_625 + 100);
        for dr in [DataRate::SPS8, DataRate::SPS128, DataRate::SPS860] {
            let nominal = 1_000_000
                / match dr {
                    DataRate::SPS8 => 8,
                    DataRate::SPS128 => 128,
                    DataRate::SPS860 => 860,
                    _ => unreachable!(),
                };
            assert!(conversion_period_us(dr) > nominal);
        }
    }

    // ---- multi-device mock tests (host-side std allowed in tests) ----
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct MockErr;
    impl embedded_hal::i2c::Error for MockErr {
        fn kind(&self) -> embedded_hal::i2c::ErrorKind { embedded_hal::i2c::ErrorKind::Other }
    }

    struct MockI2c {
        /// (addr, bytes) for every write, in order.
        writes: std::vec::Vec<(u8, std::vec::Vec<u8>)>,
        /// canned conversion result per address.
        canned: [(u8, i16); 4],
        config_reads: usize,
        /// Fail the nth `write` call (0-based) with `MockErr`; `None` disables.
        fail_write: Option<usize>,
        write_count: usize,
    }
    impl MockI2c {
        fn new() -> Self {
            Self {
                writes: std::vec::Vec::new(),
                canned: [(0x48, 100), (0x49, -200), (0x4A, 300), (0x4B, i16::MIN)],
                config_reads: 0,
                fail_write: None,
                write_count: 0,
            }
        }
        fn canned(&self, addr: u8) -> i16 {
            self.canned.iter().find(|(a, _)| *a == addr).map(|(_, v)| *v).unwrap_or(0)
        }
        fn config_writes(&self) -> usize {
            self.writes.iter().filter(|(_, b)| !b.is_empty() && b[0] == REG_CONFIG).count()
        }
    }
    impl embedded_hal::i2c::ErrorType for MockI2c {
        type Error = MockErr;
    }
    impl embedded_hal::i2c::I2c for MockI2c {
        fn read(&mut self, _a: u8, _b: &mut [u8]) -> Result<(), Self::Error> { Ok(()) }
        fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
            if self.fail_write == Some(self.write_count) {
                self.write_count += 1;
                return Err(MockErr);
            }
            self.write_count += 1;
            self.writes.push((addr, bytes.to_vec()));
            Ok(())
        }
        fn write_read(&mut self, addr: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
            match write {
                [REG_CONVERSION] => {
                    read.copy_from_slice(&self.canned(addr).to_be_bytes()[..read.len().min(2)]);
                    Ok(())
                }
                [REG_CONFIG] => {
                    self.config_reads += 1;
                    // OS=1 (complete) + preserve nothing else; tests only need OS bit.
                    read[0] = 0x80;
                    if read.len() > 1 { read[1] = 0; }
                    Ok(())
                }
                _ => Ok(()),
            }
        }
        fn transaction(&mut self, _a: u8, _o: &mut [embedded_hal::i2c::Operation<'_>]) -> Result<(), Self::Error> { Ok(()) }
    }

    fn dev(address: Address, gain: Gain, dr: DataRate, mux: Mux) -> DeviceConfig {
        DeviceConfig::new(address, Config::default().with_gain(gain).with_data_rate(dr), mux)
    }

    #[test]
    fn multi_partial_start_leaves_earlier_devices_running() {
        // Failure semantics proof: if the 3rd device's START write fails,
        // devices 0-1 remain started (their CONFIG writes are on the bus),
        // device 2+ never started, no rollback, error names device 2/SDA.
        let mut bus = MockI2c::new();
        bus.fail_write = Some(2);
        let mut m = MultiADS1115::new(
            bus,
            [
                dev(Address::Ground, Gain::V2_048, DataRate::SPS128, Mux::A0),
                dev(Address::VDD, Gain::V2_048, DataRate::SPS128, Mux::A1),
                dev(Address::SDA, Gain::V2_048, DataRate::SPS128, Mux::A2),
            ],
        )
        .unwrap();
        let err = m.start_all([Mux::A0, Mux::A1, Mux::A2]).unwrap_err();
        match err {
            MultiError::I2c { index: 2, address: Address::SDA, .. } => {}
            other => panic!("expected I2c error at device 2/SDA, got {other:?}"),
        }
        // Devices 0-1 started (CONFIG on bus), device 2 absent: partial start.
        let started: std::vec::Vec<u8> = m
            .i2c
            .writes
            .iter()
            .filter(|(_, b)| !b.is_empty() && b[0] == REG_CONFIG)
            .map(|(a, _)| *a)
            .collect();
        assert_eq!(started.as_slice(), &[0x48, 0x49]);
    }

    #[test]
    fn multi_rejects_duplicate_addresses() {        let r = MultiADS1115::new(MockI2c::new(), [
            dev(Address::Ground, Gain::V6_144, DataRate::SPS128, Mux::A0),
            dev(Address::Ground, Gain::V2_048, DataRate::SPS860, Mux::A1),
        ]);
        assert!(matches!(r, Err(MultiError::DuplicateAddress { first: 0, second: 1, .. })));
    }

    #[test]
    fn multi_parallel_start_before_read_and_ordering() {
        let mut m = MultiADS1115::new(MockI2c::new(), [
            dev(Address::Ground, Gain::V6_144, DataRate::SPS128, Mux::A0),
            dev(Address::VDD, Gain::V2_048, DataRate::SPS860, Mux::A1),
            dev(Address::SDA, Gain::V1_024, DataRate::SPS250, Mux::A2),
            dev(Address::SCL, Gain::V0_512, DataRate::SPS64, Mux::A3),
        ]).unwrap();
        let s = m.capture_parallel([Mux::A0, Mux::A1, Mux::A2, Mux::A3]).unwrap();
        // Identity + ordering preserved.
        assert_eq!((s[0].address, s[0].value), (Address::Ground, 100));
        assert_eq!((s[1].address, s[1].value), (Address::VDD, -200));
        assert_eq!((s[2].address, s[2].value), (Address::SDA, 300));
        assert_eq!((s[3].address, s[3].value), (Address::SCL, i16::MIN));
        // All 4 STARTs (CONFIG writes) precede any CONVERSION read: CONFIG reads
        // happen only in read phase; count CONFIG writes == 4 exactly.
        assert_eq!(m.i2c.config_writes(), 4);
        // Independent configs: distinct gain bits per device in CONFIG writes.
        let cfgs: std::vec::Vec<[u8; 2]> = m.i2c.writes.iter()
            .filter(|(_, b)| b[0] == REG_CONFIG)
            .map(|(_, b)| [b[1], b[2]]).collect();
        assert_eq!(cfgs.len(), 4);
        assert!(cfgs[0][0] & 0x0E != cfgs[1][0] & 0x0E);
        // MUX bits follow requested order A0..A3.
        for (i, c) in cfgs.iter().enumerate() {
            assert_eq!((c[0] >> 4) & 7, (0b100 + i as u8) & 7);
        }
    }

    #[test]
    fn multi_diff_and_invalid() {
        let mut m = MultiADS1115::new(MockI2c::new(), [
            dev(Address::Ground, Gain::V2_048, DataRate::SPS128, Mux::A0N1),
            dev(Address::VDD, Gain::V2_048, DataRate::SPS128, Mux::A0N3),
        ]).unwrap();
        let d = m.capture_parallel_diff([Mux::A0N1, Mux::A0N3]).unwrap();
        assert_eq!((d[0].negative, d[0].magnitude), (false, 100));
        assert_eq!((d[1].negative, d[1].magnitude), (true, 200));
        assert!(matches!(m.start_conversion(9, Mux::A0), Err(MultiError::InvalidIndex(9))));
        // Raw-bits helper rejects out-of-range instead of masking.
        let mut single = ADS1115::new(Address::Ground, MockI2c::new(), Config::default());
        assert!(matches!(single.read_channel_raw_bits(8), Err(MuxOrI2c::Mux(MuxError(8)))));
        assert!(matches!(single.read_channel_raw_bits(255), Err(MuxOrI2c::Mux(_))));
        assert!(single.read_channel_raw_bits(4).is_ok());
    }

    #[test]
    fn multi_continuous_writes_once() {
        let mut m = MultiADS1115::new(MockI2c::new(), [
            dev(Address::Ground, Gain::V2_048, DataRate::SPS128, Mux::A0),
            dev(Address::VDD, Gain::V2_048, DataRate::SPS128, Mux::A1),
        ]).unwrap();
        m.start_continuous_all([Mux::A0, Mux::A1]).unwrap();
        // MODE=0, OS=0 on both.
        for (_, b) in m.i2c.writes.iter().filter(|(_, b)| b[0] == REG_CONFIG) {
            assert_eq!(b[1] & 1, 0);
            assert_eq!(b[1] & 0x80, 0);
        }
        let before = m.i2c.config_writes();
        // Timed continuous read: delay called once with the max period, CONFIG
        // never rewritten, and — critically — no OS-bit polling (OS reads 0 in
        // continuous mode, so polling would hang forever).
        let mut delays = std::vec::Vec::new();
        let s = m.next_all_timed(|us| delays.push(us)).unwrap();
        assert_eq!(m.i2c.config_writes(), before, "next_all must not rewrite CONFIG");
        assert_eq!(delays.len(), 1, "single shared delay, not per-device");
        assert_eq!(delays[0], conversion_period_us(DataRate::SPS128));
        assert_eq!(m.i2c.config_reads, 0, "no CONFIG polling in continuous mode");
        assert_eq!(s[0].value, 100);
        assert_eq!(s[1].value, -200);
    }

    struct MockBlockingRdy {
        waited: std::cell::Cell<u32>,
        armed: std::cell::Cell<u32>,
    }
    impl BlockingReady for MockBlockingRdy {
        type Error = MockErr;
        fn arm(&mut self, _pol: ConversionReadyPolarity) {
            self.armed.set(self.armed.get() + 1);
        }
        fn wait_for_ready(&mut self, _pol: ConversionReadyPolarity) -> Result<(), Self::Error> {
            // Mirrors latched hardware: the wait observes the edge only if the
            // detector was armed first.
            assert!(self.armed.get() > 0, "wait without arm");
            self.waited.set(self.waited.get() + 1);
            Ok(())
        }
    }

    #[test]
    fn sync_continuous_uses_rdy_or_delay_not_os_poll() {
        let mut adc = ADS1115::new(Address::Ground, MockI2c::new(), Config::default());
        // RDY precondition: enable conversion-ready mode first (also covers the
        // debug_assert_rdy guard — default config would now panic by design).
        adc.enable_conversion_ready_pin().unwrap();
        assert!(adc.config.is_conversion_ready());
        adc.start_continuous(Mux::A0).unwrap();
        // Blocking variant: waits on RDY, reads conversion, never polls CONFIG.
        let mut rdy = MockBlockingRdy { waited: std::cell::Cell::new(0), armed: std::cell::Cell::new(0) };
        let v = adc.next_conversion_blocking(&mut rdy).unwrap();
        assert_eq!(v, 100);
        assert_eq!(rdy.waited.get(), 1);
        assert_eq!(rdy.armed.get(), 1, "single-ADC blocking path re-arms every sample");
        assert_eq!(adc.i2c.config_reads, 0, "OS polling invalid in continuous mode");
        // Timed variant: delay called once with the data-rate period.
        let mut delays = std::vec::Vec::new();
        let v = adc.next_conversion_timed(|us| delays.push(us)).unwrap();
        assert_eq!(v, 100);
        assert_eq!(delays.as_slice(), &[conversion_period_us(DataRate::SPS860)]);
        assert_eq!(adc.i2c.config_reads, 0, "OS polling invalid in continuous mode");
        // Static guard: sync continuous bodies must not reference the OS poll.
        // (There is deliberately no multi-device blocking RDY API: sequential
        // blocking waits cannot observe N independent ~8µs pulses. The safe
        // multi paths are next_all_event() and next_all_timed().)
        let src = include_str!("lib.rs");
        for name in ["fn next_conversion_blocking", "fn next_conversion_timed", "fn next_all_timed"] {
            let i = src.find(name).expect(name);
            let body = &src[i..src[i..].find("\n    }\n").map(|e| i + e).unwrap_or(src.len())];
            assert!(!body.contains("wait_complete_sync") && !body.contains("multi_wait"),
                "{name} must not OS-poll in continuous mode");
        }
        // The needle is assembled at runtime so this very assertion's source
        // text cannot self-match the forbidden definition.
        let needle = std::string::String::from("pub fn next") + "_all_blocking";
        assert!(!src.contains(&needle[..]),
            "sequential blocking multi-RDY waits were removed as pulse-unsafe");
    }
}
