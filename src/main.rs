use std::f64::consts::PI;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use std::io::{stdout, Write};

use rand::Rng;
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use crossterm::event::{self, Event, KeyCode, KeyModifiers, KeyEventKind};
use crossterm::terminal::{self, ClearType};
use crossterm::{cursor, execute};

const MAX_CYLINDERS: usize = 16;
const ATMOSPHERIC_PRESSURE: f64 = 101325.0; // Pascals

// === ENGINE PROFILES ===

#[derive(Clone)]
pub struct EngineProfile {
    pub name: &'static str,
    pub cylinders: usize,
    pub phases: [f64; MAX_CYLINDERS],
    pub exhaust_res: f64,
    pub exhaust_decay: f64,
    pub intake_res: f64,
    pub intake_decay: f64,
    pub rev_limit: f64,
    pub limiter_drop_rpm: f64, 
    pub vtec_rpm: f64,         
    pub vtec_multiplier: f64,  
    pub header_delay_base: f64,
    pub header_delay_spread: f64,
    pub bank_delay_offset: f64,
    pub idle_rpm: f64,
}

pub const PROFILES: [EngineProfile; 8] = [
    EngineProfile {
        name: "Inline 4 (Tuner)",
        cylinders: 4,
        phases: [0.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        exhaust_res: 180.0, exhaust_decay: 80.0, intake_res: 200.0, intake_decay: 15.0,
        rev_limit: 7500.0, limiter_drop_rpm: 150.0, vtec_rpm: 20000.0, vtec_multiplier: 1.0, 
        header_delay_base: 1.0, header_delay_spread: 0.5, bank_delay_offset: 0.0, idle_rpm: 900.0,
    },
    EngineProfile {
        name: "Inline 6 (2JZ)",
        cylinders: 6,
        phases: [0.0, 0.6666, 1.3333, 2.0, 2.6666, 3.3333, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        exhaust_res: 220.0, exhaust_decay: 100.0, intake_res: 240.0, intake_decay: 20.0,
        rev_limit: 8000.0, limiter_drop_rpm: 200.0, vtec_rpm: 20000.0, vtec_multiplier: 1.0, 
        header_delay_base: 1.2, header_delay_spread: 0.3, bank_delay_offset: 0.0, idle_rpm: 800.0,
    },
    EngineProfile {
        name: "V8 Cross-Plane (Muscle)",
        cylinders: 8,
        phases: [0.0, 0.5, 1.0, 1.5, 2.5, 2.0, 3.5, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        exhaust_res: 130.0, exhaust_decay: 120.0, intake_res: 160.0, intake_decay: 15.0,
        rev_limit: 6500.0, limiter_drop_rpm: 200.0, vtec_rpm: 20000.0, vtec_multiplier: 1.0, 
        header_delay_base: 2.0, header_delay_spread: 1.5, bank_delay_offset: 1.2, idle_rpm: 700.0,
    },
    EngineProfile {
        name: "V8 Flat-Plane (Supercar)",
        cylinders: 8,
        phases: [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        exhaust_res: 280.0, exhaust_decay: 140.0, intake_res: 180.0, intake_decay: 18.0,
        rev_limit: 8500.0, limiter_drop_rpm: 120.0, vtec_rpm: 20000.0, vtec_multiplier: 1.0, 
        header_delay_base: 0.8, header_delay_spread: 0.2, bank_delay_offset: 0.8, idle_rpm: 1000.0,
    },
    EngineProfile {
        name: "V10 (LFA-style)",
        cylinders: 10,
        phases: [0.0, 0.4, 0.8, 1.2, 1.6, 2.0, 2.4, 2.8, 3.2, 3.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        exhaust_res: 340.0, exhaust_decay: 150.0, intake_res: 140.0, intake_decay: 25.0,
        rev_limit: 9000.0, limiter_drop_rpm: 100.0, vtec_rpm: 20000.0, vtec_multiplier: 1.0, 
        header_delay_base: 0.5, header_delay_spread: 0.15, bank_delay_offset: 0.4, idle_rpm: 1000.0,
    },
    EngineProfile {
        name: "V12 (SVJ-style)",
        cylinders: 12,
        phases: [0.0, 0.3333, 0.6666, 1.0, 1.3333, 1.6666, 2.0, 2.3333, 2.6666, 3.0, 3.3333, 3.6666, 0.0, 0.0, 0.0, 0.0],
        exhaust_res: 380.0, exhaust_decay: 160.0, intake_res: 150.0, intake_decay: 30.0,
        rev_limit: 8700.0, limiter_drop_rpm: 100.0, vtec_rpm: 20000.0, vtec_multiplier: 1.0, 
        header_delay_base: 0.4, header_delay_spread: 0.1, bank_delay_offset: 0.3, idle_rpm: 950.0,
    },
    EngineProfile {
        name: "V6 1.6L (2026 F1)",
        cylinders: 6,
        phases: [0.0, 0.6666, 1.3333, 2.0, 2.6666, 3.3333, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        exhaust_res: 550.0, exhaust_decay: 35.0, intake_res: 380.0, intake_decay: 12.0,
        rev_limit: 12500.0, limiter_drop_rpm: 50.0, vtec_rpm: 20000.0, vtec_multiplier: 1.0, 
        header_delay_base: 0.1, header_delay_spread: 0.02, bank_delay_offset: 0.05, idle_rpm: 3500.0,     
    },
    EngineProfile {
        name: "Inline 4 (K20 Type R)",
        cylinders: 4,
        phases: [0.0, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        exhaust_res: 185.0, exhaust_decay: 55.0, 
        intake_res: 300.0, intake_decay: 25.0,   
        rev_limit: 8400.0, 
        limiter_drop_rpm: 60.0,  
        vtec_rpm: 4000.0,        
        vtec_multiplier: 1.6,    
        header_delay_base: 0.7, header_delay_spread: 0.1, bank_delay_offset: 0.0, idle_rpm: 900.0,
    }
];

// === SHARED TELEMETRY STATE ===

pub struct SharedEngineState {
    pub throttle: AtomicU64,
    pub rpm: AtomicU64,
    pub profile_idx: AtomicUsize,
    pub displacement: AtomicU64,
    pub pressures: [AtomicU64; MAX_CYLINDERS],
    pub exhaust_flows: [AtomicU64; MAX_CYLINDERS],
    pub intake_flows: [AtomicU64; MAX_CYLINDERS],
    pub rod_knock: AtomicBool,
    pub piston_slap: AtomicBool,
    pub limiter_active: AtomicBool,
    pub vtec_active: AtomicBool,
    pub dyno_load: AtomicBool, 
}

impl SharedEngineState {
    pub fn new() -> Self {
        Self {
            throttle: AtomicU64::new(0f64.to_bits()),
            rpm: AtomicU64::new(0f64.to_bits()),
            profile_idx: AtomicUsize::new(7), 
            displacement: AtomicU64::new(1.0_f64.to_bits()),
            pressures: std::array::from_fn(|_| AtomicU64::new(ATMOSPHERIC_PRESSURE.to_bits())),
            exhaust_flows: std::array::from_fn(|_| AtomicU64::new(0f64.to_bits())),
            intake_flows: std::array::from_fn(|_| AtomicU64::new(0f64.to_bits())),
            rod_knock: AtomicBool::new(false),
            piston_slap: AtomicBool::new(false),
            limiter_active: AtomicBool::new(false),
            vtec_active: AtomicBool::new(false),
            dyno_load: AtomicBool::new(true), 
        }
    }
}

// === 1. KINEMATIC ENGINE PHYSICS (Restored for pristine audio quality) ===

pub struct EngineSolver {
    pub base_crank_radius: f64,
    pub base_rod_length: f64,
    pub base_crank_inertia: f64,
    pub base_piston_mass: f64,
    pub base_bore: f64,
    pub compression_ratio: f64,

    pub num_cylinders: usize,
    pub displacement_scale: f64,
    pub profile_idx: usize,
    pub rev_limit: f64,
    pub limiter_drop_rpm: f64,
    pub vtec_rpm: f64,
    pub vtec_multiplier: f64,
    pub idle_rpm: f64,
    
    pub rev_limiter_active: bool,

    crank_radius: f64,
    rod_length: f64,
    piston_mass: f64,
    piston_area: f64,
    clearance_volume: f64,

    pub cylinder_pressure: [f64; MAX_CYLINDERS],
    pub unburnt_fuel: [f64; MAX_CYLINDERS], // Tracks fuel for Spark Cut Pops!
    prev_volume: [Option<f64>; MAX_CYLINDERS],
    prev_cycle_angle: [f64; MAX_CYLINDERS],
    phase_offsets: [f64; MAX_CYLINDERS],
    
    rod_knock_env: [f64; MAX_CYLINDERS],
    piston_slap_env: [f64; MAX_CYLINDERS],
    
    pub crank_angle: f64,
    pub angular_velocity: f64,
}

impl EngineSolver {
    pub fn new() -> Self {
        Self {
            base_crank_radius: 0.04, base_rod_length: 0.13, base_crank_inertia: 0.15,
            base_piston_mass: 0.4, base_bore: 0.08, compression_ratio: 10.5, 
            num_cylinders: 8, displacement_scale: 1.0, profile_idx: 999,
            rev_limit: 8000.0, limiter_drop_rpm: 250.0, vtec_rpm: 20000.0, vtec_multiplier: 1.0, 
            idle_rpm: 1000.0, rev_limiter_active: false,
            crank_radius: 0.0, rod_length: 0.0, piston_mass: 0.0, piston_area: 0.0, clearance_volume: 0.0,

            cylinder_pressure: [ATMOSPHERIC_PRESSURE; MAX_CYLINDERS],
            unburnt_fuel: [0.0; MAX_CYLINDERS],
            prev_volume: [None; MAX_CYLINDERS], prev_cycle_angle: [0.0; MAX_CYLINDERS], phase_offsets: [0.0; MAX_CYLINDERS],
            
            rod_knock_env: [0.0; MAX_CYLINDERS], piston_slap_env: [0.0; MAX_CYLINDERS],
            crank_angle: 0.0, angular_velocity: 800.0 * (2.0 * PI / 60.0),
        }
    }

    pub fn reconfigure(&mut self, profile: &EngineProfile, scale: f64, profile_idx: usize) {
        self.profile_idx = profile_idx;
        self.num_cylinders = profile.cylinders;
        self.displacement_scale = scale;
        self.rev_limit = profile.rev_limit;
        self.limiter_drop_rpm = profile.limiter_drop_rpm;
        self.vtec_rpm = profile.vtec_rpm;
        self.vtec_multiplier = profile.vtec_multiplier;
        self.idle_rpm = profile.idle_rpm;
        self.rev_limiter_active = false;

        let s = scale.cbrt(); 
        self.crank_radius = self.base_crank_radius * s;
        self.rod_length = self.base_rod_length * s;
        self.piston_area = PI * (self.base_bore * s / 2.0).powi(2);
        
        let swept_volume = self.piston_area * (2.0 * self.crank_radius);
        self.clearance_volume = swept_volume / (self.compression_ratio - 1.0);
        self.piston_mass = self.base_piston_mass * scale;

        for i in 0..MAX_CYLINDERS {
            self.cylinder_pressure[i] = ATMOSPHERIC_PRESSURE;
            self.unburnt_fuel[i] = 0.0;
            self.prev_volume[i] = None;
            self.prev_cycle_angle[i] = 0.0;
            self.rod_knock_env[i] = 0.0;
            self.piston_slap_env[i] = 0.0;
            self.phase_offsets[i] = if i < self.num_cylinders { profile.phases[i] * PI } else { 0.0 };
        }
    }

    pub fn step(&mut self, dt: f64, throttle: f64, rod_knock: bool, piston_slap: bool, load_active: bool) -> ([f64; MAX_CYLINDERS], [f64; MAX_CYLINDERS], f64, f64, bool) {
        let r = self.crank_radius;
        let l = self.rod_length;
        let mut total_combustion_torque = 0.0;
        
        let mut total_rk_burst = 0.0;
        let mut total_ps_burst = 0.0;
        
        let mut exhaust_flows = [0.0; MAX_CYLINDERS];
        let mut intake_flows = [0.0; MAX_CYLINDERS];
        let mut rng = rand::thread_rng();

        // REV LIMITER LOGIC
        let rev_limit_rads = self.rev_limit * (2.0 * PI / 60.0);
        let bounce_drop_rads = self.limiter_drop_rpm * (2.0 * PI / 60.0);
        
        if self.angular_velocity > rev_limit_rads {
            self.rev_limiter_active = true;
        } else if self.angular_velocity < (rev_limit_rads - bounce_drop_rads) {
            self.rev_limiter_active = false;
        }

        // VTEC LOGIC
        let is_vtec = self.angular_velocity > self.vtec_rpm * (2.0 * PI / 60.0);
        let vtec_mod = if is_vtec { self.vtec_multiplier } else { 1.0 };

        let intake_manifold_pressure = ATMOSPHERIC_PRESSURE * (0.2 + throttle * 0.8);
        
        let intake_flow_rate = 1.0 - (-dt / (0.002 / vtec_mod)).exp();
        let exhaust_flow_rate = 1.0 - (-dt / (0.0005 / vtec_mod)).exp();

        // SIMULATED LOAD (Fixes the hyper-revving issue)
        let mut base_inertia = self.base_crank_inertia * self.num_cylinders as f64 * self.displacement_scale;
        let mut load_drag = 0.0;
        if load_active {
            base_inertia += 1.5 * self.displacement_scale; // Mass of drivetrain
            load_drag = -0.00025 * self.angular_velocity.powi(2) * self.displacement_scale; // High speed wind resistance
        }
        let total_inertia = base_inertia;

        for i in 0..self.num_cylinders {
            let theta = self.crank_angle + self.phase_offsets[i];
            let sin_t = theta.sin(); let cos_t = theta.cos();
            let rod_term = (l * l - r * r * sin_t * sin_t).sqrt();
            let dx_dtheta = -r * sin_t - ((r * r * sin_t * cos_t) / rod_term);

            let current_x = r * cos_t + rod_term;
            let volume = self.clearance_volume + self.piston_area * ((r + l) - current_x);

            let mut cycle_angle = theta % (4.0 * PI);
            if cycle_angle < 0.0 { cycle_angle += 4.0 * PI; }

            // --- 1. GAS DYNAMICS ---
            if let Some(prev_vol) = self.prev_volume[i] {
                self.cylinder_pressure[i] *= (prev_vol / volume).powf(1.4);
            }

            let is_intake = cycle_angle < 1.05 * PI || cycle_angle > 3.9 * PI; 
            let is_exhaust = cycle_angle > 2.9 * PI || cycle_angle < 0.1 * PI; 

            if is_intake {
                let diff = self.cylinder_pressure[i] - intake_manifold_pressure;
                let flow = diff * intake_flow_rate;
                self.cylinder_pressure[i] -= flow;

                let raw_suck = flow * 0.00001; 
                let turbulence = raw_suck.abs() * 0.8 * rng.gen_range(-1.0..1.0); 
                intake_flows[i] = raw_suck + turbulence;

                // Accumulate fuel for the spark cut!
                if throttle > 0.05 {
                    self.unburnt_fuel[i] += throttle * dt;
                }
            }

            let mut pop_burst = 0.0;

            if is_exhaust {
                let diff = self.cylinder_pressure[i] - ATMOSPHERIC_PRESSURE;
                let flow = diff * exhaust_flow_rate;
                self.cylinder_pressure[i] -= flow;

                // SPARK CUT POP: If fuel is in the cylinder during the exhaust stroke, ignite it in the pipe!
                if self.unburnt_fuel[i] > 0.0001 {
                    pop_burst += self.unburnt_fuel[i] * 120.0 * rng.gen_range(-1.0..1.0); // Explosive burst
                    self.unburnt_fuel[i] = 0.0; // Consume the fuel
                }

                let raw_pulse = flow * 0.00001;
                let turbulence = raw_pulse.abs() * 0.5 * rng.gen_range(-1.0..1.0);
                exhaust_flows[i] = raw_pulse + turbulence + pop_burst;
            }

            let prev_ca = self.prev_cycle_angle[i];
            
            // --- 2. COMBUSTION PULSE ---
            if prev_ca < 2.0 * PI && cycle_angle >= 2.0 * PI {
                if !self.rev_limiter_active {
                    let jitter = rng.gen_range(0.95..1.05); 
                    let combustion_multiplier = 1.0 + (20.0 * (0.1 + throttle * 0.9) * jitter * vtec_mod);
                    self.cylinder_pressure[i] *= combustion_multiplier;
                    
                    // Fuel was burnt successfully, reset tracker
                    self.unburnt_fuel[i] = 0.0;
                }
                // NOTE: If rev limiter IS active, we don't multiply pressure (no spark), 
                // and the fuel stays in the unburnt_fuel tracker to be popped on the exhaust stroke!
            }

            // --- 3. MECHANICAL DAMAGE EXCITERS ---
            self.rod_knock_env[i] *= (-dt / 0.002).exp();
            self.piston_slap_env[i] *= (-dt / 0.001).exp();
            
            let crossed_0 = prev_ca > 3.0 * PI && cycle_angle < 1.0 * PI;
            let crossed_pi = prev_ca < PI && cycle_angle >= PI;
            let crossed_2pi = prev_ca < 2.0 * PI && cycle_angle >= 2.0 * PI;
            let crossed_3pi = prev_ca < 3.0 * PI && cycle_angle >= 3.0 * PI;
            
            if rod_knock {
                if crossed_2pi { self.rod_knock_env[i] = 1.0 + throttle * 1.5; } 
                else if crossed_0 { self.rod_knock_env[i] = 0.4 + throttle * 0.5; }
            }
            if piston_slap {
                if crossed_0 || crossed_pi || crossed_2pi || crossed_3pi { self.piston_slap_env[i] = 0.6 + throttle * 0.4; }
            }
            
            total_rk_burst += self.rod_knock_env[i] * rng.gen_range(-1.0..1.0);
            total_ps_burst += self.piston_slap_env[i] * rng.gen_range(-1.0..1.0);

            // --- 4. STATE UPDATES ---
            self.prev_volume[i] = Some(volume);
            self.prev_cycle_angle[i] = cycle_angle;

            let net_pressure = self.cylinder_pressure[i] - ATMOSPHERIC_PRESSURE;
            let cylinder_pressure_force = net_pressure * self.piston_area;
            total_combustion_torque += -cylinder_pressure_force * dx_dtheta;
        }

        let friction_torque = -0.02 * self.num_cylinders as f64 * self.displacement_scale * self.angular_velocity;

        let rpm_error = (self.idle_rpm * 2.0 * PI / 60.0) - self.angular_velocity;
        let idle_torque = if rpm_error > 0.0 { rpm_error * 5.0 * self.num_cylinders as f64 * self.displacement_scale } else { 0.0 };

        let total_torque = total_combustion_torque + friction_torque + load_drag + idle_torque;

        self.angular_velocity += (total_torque / total_inertia) * dt;
        if self.angular_velocity < 0.0 { self.angular_velocity = 0.0; } 
        self.crank_angle += self.angular_velocity * dt;
        if self.crank_angle > 2000.0 * PI { self.crank_angle -= 2000.0 * PI; }

        (exhaust_flows, intake_flows, total_rk_burst, total_ps_burst, is_vtec)
    }
}

// === 2. ACOUSTIC FILTERS ===

struct DamageResonator { ir: Vec<f64>, history: Vec<f64>, ptr: usize }
impl DamageResonator {
    fn new(sample_rate: f64, base_freq: f64, decay_speed: f64, length: usize, is_knock: bool) -> Self {
        let mut ir = vec![0.0; length];
        let mut rng = rand::thread_rng();
        let mut lp = 0.0; let mut energy = 0.0;
        for i in 0..length {
            let envelope = (-(i as f64) / decay_speed).exp();
            let t = i as f64 / sample_rate;
            let wave = if is_knock {
                (t * base_freq * 2.0 * PI).cos() + (t * base_freq * 1.618 * 2.0 * PI).cos() * 0.5 + rng.gen_range(-1.0..1.0) * 0.1 
            } else {
                (t * base_freq * 2.0 * PI).cos() + (t * base_freq * 1.32 * 2.0 * PI).cos() * 0.5 + rng.gen_range(-1.0..1.0) * 0.8 
            };
            lp += 0.5 * (wave * envelope - lp); ir[i] = lp; energy += lp.abs();
        }
        if energy > 0.0 { let scale = 20.0 / energy; for val in ir.iter_mut() { *val *= scale; } }
        Self { ir, history: vec![0.0; length], ptr: 0 }
    }
    fn process(&mut self, input: f64) -> f64 {
        self.history[self.ptr] = input;
        let mut sum = 0.0; let len = self.ir.len();
        for i in 0..len { sum += self.history[(self.ptr + len - i) % len] * self.ir[i]; }
        self.ptr = (self.ptr + 1) % len; sum
    }
}

struct ConvolutionFilter { ir: Vec<f64>, history: Vec<f64>, ptr: usize }
impl ConvolutionFilter {
    fn new(sample_rate: f64, resonance_freq: f64, decay_speed: f64, length: usize) -> Self {
        let mut ir = vec![0.0; length]; let mut rng = rand::thread_rng(); let mut lp = 0.0;
        for i in 0..length {
            let noise = rng.gen_range(-1.0..1.0);
            let envelope = (-(i as f64) / decay_speed).exp(); 
            let resonance = (i as f64 * 2.0 * PI * resonance_freq / sample_rate).cos(); 
            lp += 0.3 * ((noise * envelope * 0.5 + resonance * envelope * 0.5) - lp); ir[i] = lp;
        }
        Self { ir, history: vec![0.0; length], ptr: 0 }
    }
    fn process(&mut self, input: f64) -> f64 {
        self.history[self.ptr] = input; let mut sum = 0.0; let len = self.ir.len();
        for i in 0..len { sum += self.history[(self.ptr + len - i) % len] * self.ir[i]; }
        self.ptr = (self.ptr + 1) % len; sum
    }
}

struct HeaderDelay { buffer: Vec<f64>, ptr: usize }
impl HeaderDelay {
    fn new(delay_samples: usize) -> Self { Self { buffer: vec![0.0; delay_samples.max(1)], ptr: 0 } }
    fn process(&mut self, input: f64) -> f64 { let output = self.buffer[self.ptr]; self.buffer[self.ptr] = input; self.ptr = (self.ptr + 1) % self.buffer.len(); output }
}
struct AutoGain { peak: f64 }
impl AutoGain {
    fn new() -> Self { Self { peak: 1.0 } }
    fn process(&mut self, input: f64) -> f64 { let abs_i = input.abs(); if abs_i > self.peak { self.peak = abs_i; } else { self.peak = self.peak * 0.99995 + 1.0 * 0.00005; } input / self.peak }
}
struct DcBlocker { x_prev: f64, y_prev: f64 }
impl DcBlocker {
    fn new() -> Self { Self { x_prev: 0.0, y_prev: 0.0 } }
    fn process(&mut self, x: f64) -> f64 { let y = x - self.x_prev + 0.995 * self.y_prev; self.x_prev = x; self.y_prev = y; y }
}

// === 3. AUDIO THREAD ===

fn run_audio_stream(state: Arc<SharedEngineState>) -> cpal::Stream {
    let host = cpal::default_host();
    let device = host.default_output_device().expect("No output device found!");
    let config = device.default_output_config().unwrap();
    let sample_rate = config.sample_rate().0 as f64;
    let channels = config.channels() as usize;

    let mut engine = EngineSolver::new();
    let mut header_delays: Vec<HeaderDelay> = vec![];
    
    let mut exhaust_pipe = ConvolutionFilter::new(sample_rate, 150.0, 120.0, 1024);
    let mut intake_box = ConvolutionFilter::new(sample_rate, 240.0, 15.0, 512);
    
    let mut rk_resonator = DamageResonator::new(sample_rate, 450.0, 1500.0, 2048, true);
    let mut ps_resonator = DamageResonator::new(sample_rate, 1800.0, 400.0, 1024, false);
    
    let mut agc = AutoGain::new();
    let mut dc_block = DcBlocker::new();
    let dt = 1.0 / sample_rate;

    let stream = device.build_output_stream(
        &config.into(),
        move |data: &mut [f32], _: &cpal::OutputCallbackInfo| {
            let target_profile = state.profile_idx.load(Ordering::Relaxed);
            let target_disp = f64::from_bits(state.displacement.load(Ordering::Relaxed));
            
            if target_profile != engine.profile_idx || target_disp != engine.displacement_scale || header_delays.is_empty() {
                let profile = &PROFILES[target_profile];
                engine.reconfigure(profile, target_disp, target_profile);
                header_delays.clear();
                
                exhaust_pipe = ConvolutionFilter::new(sample_rate, profile.exhaust_res, profile.exhaust_decay, 1024);
                intake_box = ConvolutionFilter::new(sample_rate, profile.intake_res, profile.intake_decay, 512);
                
                for i in 0..profile.cylinders {
                    let bank_pos = (i % 2) as f64;
                    let imperfection = ((i as f64) * 1.618).sin() * profile.header_delay_spread;
                    let delay_ms = profile.header_delay_base + (bank_pos * profile.bank_delay_offset) + imperfection; 
                    header_delays.push(HeaderDelay::new((delay_ms / 1000.0 * sample_rate).max(1.0) as usize));
                }
            }

            let profile = &PROFILES[target_profile];
            let throttle = f64::from_bits(state.throttle.load(Ordering::Relaxed));
            let rk_active = state.rod_knock.load(Ordering::Relaxed);
            let ps_active = state.piston_slap.load(Ordering::Relaxed);
            let load_active = state.dyno_load.load(Ordering::Relaxed);
            
            let mut last_ex = [0.0; MAX_CYLINDERS];
            let mut last_in = [0.0; MAX_CYLINDERS];
            let mut any_limiter = false;
            let mut current_vtec = false;

            for frame in data.chunks_mut(channels) {
                let (raw_exhausts, raw_intakes, rk_burst, ps_burst, is_vtec) = engine.step(dt, throttle, rk_active, ps_active, load_active);
                
                if engine.rev_limiter_active { any_limiter = true; }
                current_vtec = is_vtec;

                last_ex = raw_exhausts;
                last_in = raw_intakes;

                let mut mixed_exhaust = 0.0;
                let mut mixed_intake = 0.0;
                
                for i in 0..engine.num_cylinders {
                    mixed_exhaust += header_delays[i].process(raw_exhausts[i]);
                    mixed_intake += raw_intakes[i]; 
                }
                
                let ex_convolved = exhaust_pipe.process(mixed_exhaust);
                let in_convolved = intake_box.process(mixed_intake);
                
                let rk_sound = rk_resonator.process(rk_burst);
                let ps_sound = ps_resonator.process(ps_burst);
                
                let drive = 1.5 + (throttle * 4.0); 
                let vtec_volume_boost = if is_vtec { 0.6 } else { 0.0 };
                let intake_vol = 0.3 + (throttle * 0.7) + vtec_volume_boost; 
                
                let final_engine_mix = ex_convolved + (in_convolved * intake_vol);
                let blocked = dc_block.process(final_engine_mix);
                
                let overdriven = (blocked * drive).tanh() / drive.tanh();
                let final_mix = (overdriven * 0.7) + (rk_sound * 1.5) + (ps_sound * 0.8);
                let normalized = agc.process(final_mix);
                
                let sample_f32 = (normalized * 0.8).clamp(-1.0, 1.0) as f32;
                for channel in frame.iter_mut() { *channel = sample_f32; }
            }

            state.rpm.store((engine.angular_velocity * (60.0 / (2.0 * PI))).to_bits(), Ordering::Relaxed);
            state.limiter_active.store(any_limiter, Ordering::Relaxed);
            state.vtec_active.store(current_vtec, Ordering::Relaxed);

            for i in 0..profile.cylinders {
                state.pressures[i].store(engine.cylinder_pressure[i].to_bits(), Ordering::Relaxed);
                state.exhaust_flows[i].store(last_ex[i].to_bits(), Ordering::Relaxed);
                state.intake_flows[i].store(last_in[i].to_bits(), Ordering::Relaxed);
            }
        },
        |err| eprintln!("Audio error: {}", err),
        None,
    ).expect("Failed to build audio stream");

    stream.play().unwrap();
    stream
}

// === 4. TERMINAL UI ===

fn main() {
    let state = Arc::new(SharedEngineState::new());
    let _stream = run_audio_stream(state.clone());

    terminal::enable_raw_mode().unwrap();
    let mut stdout = stdout();
    execute!(stdout, terminal::Clear(ClearType::All), cursor::Hide).unwrap();

    let mut target_throttle: f64 = 0.0;
    let mut actual_throttle: f64 = 0.0;
    let frame_duration = Duration::from_millis(33);

    'main_loop: loop {
        let start_time = Instant::now();

        if event::poll(Duration::from_millis(5)).unwrap() {
            if let Event::Key(key) = event::read().unwrap() {
                if key.kind == KeyEventKind::Press {
                    match key.code {
                        KeyCode::Char('w') | KeyCode::Char('W') => target_throttle = 1.0,
                        KeyCode::Char('l') | KeyCode::Char('L') => {
                            let current = state.dyno_load.load(Ordering::Relaxed);
                            state.dyno_load.store(!current, Ordering::Relaxed);
                        }
                        KeyCode::Char('r') | KeyCode::Char('R') => {
                            let current = state.rod_knock.load(Ordering::Relaxed);
                            state.rod_knock.store(!current, Ordering::Relaxed);
                        }
                        KeyCode::Char('p') | KeyCode::Char('P') => {
                            let current = state.piston_slap.load(Ordering::Relaxed);
                            state.piston_slap.store(!current, Ordering::Relaxed);
                        }
                        KeyCode::Up => {
                            let mut p = state.profile_idx.load(Ordering::Relaxed);
                            if p < PROFILES.len() - 1 { p += 1; state.profile_idx.store(p, Ordering::Relaxed); }
                        }
                        KeyCode::Down => {
                            let mut p = state.profile_idx.load(Ordering::Relaxed);
                            if p > 0 { p -= 1; state.profile_idx.store(p, Ordering::Relaxed); }
                        }
                        KeyCode::Right => {
                            let d = (f64::from_bits(state.displacement.load(Ordering::Relaxed)) * 1.1).min(10.0);
                            state.displacement.store(d.to_bits(), Ordering::Relaxed);
                        }
                        KeyCode::Left => {
                            let d = (f64::from_bits(state.displacement.load(Ordering::Relaxed)) / 1.1).max(0.1);
                            state.displacement.store(d.to_bits(), Ordering::Relaxed);
                        }
                        KeyCode::Char('q') | KeyCode::Esc => break 'main_loop,
                        KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => break 'main_loop,
                        _ => {}
                    }
                }
            }
        } else {
            target_throttle = 0.0;
        }

        actual_throttle += (target_throttle - actual_throttle) * 0.15;
        state.throttle.store(actual_throttle.to_bits(), Ordering::Relaxed);

        let rpm = f64::from_bits(state.rpm.load(Ordering::Relaxed));
        let p_idx = state.profile_idx.load(Ordering::Relaxed);
        let profile = &PROFILES[p_idx];
        let num_cyls = profile.cylinders;
        let current_disp = f64::from_bits(state.displacement.load(Ordering::Relaxed));

        execute!(stdout, cursor::MoveTo(0, 0)).unwrap();
        write!(stdout, "🏎️   THE ANGE SYNTHESIZER (HYBRID KINEMATIC SIM) 🏎️\r\n").unwrap();
        write!(stdout, "--- Pristine Audio + Drivetrain Load & Spark Cuts ---\r\n\n").unwrap();
        write!(stdout, "[ W ]          : Rev Throttle\r\n").unwrap();
        write!(stdout, "[ Up/Down ]    : Profile       ({}/{}) {}\r\n", p_idx + 1, PROFILES.len(), profile.name).unwrap();
        write!(stdout, "[ Left/Right ] : Displacement  ({:.2}x)   \r\n", current_disp).unwrap();
        
        let load_status = if state.dyno_load.load(Ordering::Relaxed) { "ON (Slow Pull)" } else { "OFF (Neutral) " };
        let rk_status = if state.rod_knock.load(Ordering::Relaxed) { "ON " } else { "OFF" };
        let ps_status = if state.piston_slap.load(Ordering::Relaxed) { "ON " } else { "OFF" };
        
        write!(stdout, "[ L ]          : Drivetrain Load     [{}]\r\n", load_status).unwrap();
        write!(stdout, "[ R ]          : Toggle Rod Knock    [{}]\r\n", rk_status).unwrap();
        write!(stdout, "[ P ]          : Toggle Piston Slap  [{}]\r\n", ps_status).unwrap();
        write!(stdout, "[ Q / ESC ]    : Quit\r\n\n").unwrap();

        let vtec_str = if profile.vtec_rpm > 15000.0 {
            "N/A".to_string()
        } else if state.vtec_active.load(Ordering::Relaxed) {
            "██ ACTIVE ██".to_string() 
        } else {
            "STANDBY".to_string()
        };
        write!(stdout, "VTEC Status:  [{}]\r\n\n", vtec_str).unwrap();

        let rev_lim = profile.rev_limit;
        let rpm_bar_len = ((rpm / rev_lim) * 40.0).clamp(0.0, 40.0) as usize;
        let rpm_bar = "█".repeat(rpm_bar_len) + &"-".repeat(40_usize.saturating_sub(rpm_bar_len));
        
        let limiter_on = state.limiter_active.load(Ordering::Relaxed);
        if limiter_on {
            write!(stdout, "RPM:      {:05.0} [|||||||||||| SPARK CUT ||||||||||||]\r\n", rpm).unwrap();
        } else {
            write!(stdout, "RPM:      {:05.0} [{}]\r\n", rpm, rpm_bar).unwrap();
        }

        let t_bar_len = ((actual_throttle / 1.0) * 40.0).clamp(0.0, 40.0) as usize;
        let t_bar = "█".repeat(t_bar_len) + &"-".repeat(40_usize.saturating_sub(t_bar_len));
        write!(stdout, "Throttle: {:03.0}% [{}]\r\n", actual_throttle * 100.0, t_bar).unwrap();

        write!(stdout, "\r\n--- Real-Time Cylinder Telemetry ---\r\n").unwrap();
        for i in 0..num_cyls {
            let p = f64::from_bits(state.pressures[i].load(Ordering::Relaxed));
            let p_kpa = p / 1000.0;
            write!(stdout, "Cyl {:02} | Press: {:7.1} kPa \r\n", i + 1, p_kpa).unwrap();
        }
        write!(stdout, "{}", terminal::Clear(ClearType::FromCursorDown)).unwrap();

        stdout.flush().unwrap();
        let elapsed = start_time.elapsed();
        if elapsed < frame_duration { std::thread::sleep(frame_duration - elapsed); }
    }

    execute!(stdout, cursor::Show).unwrap();
    terminal::disable_raw_mode().unwrap();
}