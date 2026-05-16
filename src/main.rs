use std::f64::consts::PI;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use std::io::{stdout, Write};

use rand::Rng;
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use crossterm::event::{self, Event, KeyCode, KeyModifiers};
use crossterm::terminal::{self, ClearType};
use crossterm::{cursor, execute};

const MAX_CYLINDERS: usize = 16;
const ATMOSPHERIC_PRESSURE: f64 = 101325.0; // Pascals

// === SHARED TELEMETRY STATE ===
pub struct SharedEngineState {
    pub throttle: AtomicU64,
    pub rpm: AtomicU64,
    pub cylinders: AtomicUsize,
    pub displacement: AtomicU64,
    pub pressures: [AtomicU64; MAX_CYLINDERS],
    pub exhaust_flows: [AtomicU64; MAX_CYLINDERS],
    pub intake_flows: [AtomicU64; MAX_CYLINDERS],
}

impl SharedEngineState {
    pub fn new() -> Self {
        Self {
            throttle: AtomicU64::new(0f64.to_bits()),
            rpm: AtomicU64::new(0f64.to_bits()),
            cylinders: AtomicUsize::new(8), 
            displacement: AtomicU64::new(1.0_f64.to_bits()),
            pressures: std::array::from_fn(|_| AtomicU64::new(ATMOSPHERIC_PRESSURE.to_bits())),
            exhaust_flows: std::array::from_fn(|_| AtomicU64::new(0f64.to_bits())),
            intake_flows: std::array::from_fn(|_| AtomicU64::new(0f64.to_bits())),
        }
    }
}

// === 1. BULLETPROOF 4-STROKE PHYSICS ENGINE ===

pub struct EngineSolver {
    pub base_crank_radius: f64,
    pub base_rod_length: f64,
    pub base_crank_inertia: f64,
    pub base_piston_mass: f64,
    pub base_bore: f64,
    pub compression_ratio: f64,

    pub num_cylinders: usize,
    pub displacement_scale: f64,

    crank_radius: f64,
    rod_length: f64,
    piston_mass: f64,
    piston_area: f64,
    clearance_volume: f64,

    pub cylinder_pressure: [f64; MAX_CYLINDERS],
    prev_volume: [Option<f64>; MAX_CYLINDERS],
    prev_cycle_angle: [f64; MAX_CYLINDERS],
    phase_offsets: [f64; MAX_CYLINDERS],
    
    pub crank_angle: f64,
    pub angular_velocity: f64,
}

impl EngineSolver {
    pub fn new() -> Self {
        let mut engine = Self {
            base_crank_radius: 0.04, base_rod_length: 0.13, base_crank_inertia: 0.15,
            base_piston_mass: 0.4, base_bore: 0.08, compression_ratio: 9.0,
            num_cylinders: 8, displacement_scale: 1.0,
            crank_radius: 0.0, rod_length: 0.0, piston_mass: 0.0, piston_area: 0.0, clearance_volume: 0.0,
            cylinder_pressure: [ATMOSPHERIC_PRESSURE; MAX_CYLINDERS],
            prev_volume: [None; MAX_CYLINDERS], prev_cycle_angle: [0.0; MAX_CYLINDERS], phase_offsets: [0.0; MAX_CYLINDERS],
            crank_angle: 0.0, angular_velocity: 1000.0 * (2.0 * PI / 60.0),
        };
        engine.reconfigure(8, 1.0);
        engine
    }

    pub fn reconfigure(&mut self, cylinders: usize, scale: f64) {
        self.num_cylinders = cylinders;
        self.displacement_scale = scale;

        let s = scale.cbrt(); 
        self.crank_radius = self.base_crank_radius * s;
        self.rod_length = self.base_rod_length * s;
        self.piston_area = PI * (self.base_bore * s / 2.0).powi(2);
        
        let swept_volume = self.piston_area * (2.0 * self.crank_radius);
        self.clearance_volume = swept_volume / (self.compression_ratio - 1.0);
        self.piston_mass = self.base_piston_mass * scale;

        for i in 0..MAX_CYLINDERS {
            self.cylinder_pressure[i] = ATMOSPHERIC_PRESSURE;
            self.prev_volume[i] = None;
            self.prev_cycle_angle[i] = 0.0;
        }

        let phases: Vec<f64> = match cylinders {
            4 => vec![0.0, 3.0, 1.0, 2.0], 
            6 => vec![0.0, 3.333, 1.333, 2.666, 0.666, 2.0], 
            8 => vec![0.0, 3.5, 1.5, 1.0, 2.5, 2.0, 3.0, 0.5], 
            10 => vec![0.0, 3.6, 1.6, 1.2, 2.8, 2.0, 3.2, 2.4, 0.8, 0.4],
            12 => vec![0.0, 3.666, 1.333, 2.333, 0.666, 3.0, 1.666, 2.666, 0.333, 2.0, 1.0, 3.333], // V12 Firing Order
            _ => {
                let spacing = 4.0 / (cylinders as f64);
                (0..cylinders).map(|i| (i as f64) * spacing).collect()
            }
        };

        for i in 0..MAX_CYLINDERS {
            if i < cylinders {
                self.phase_offsets[i] = phases[i] * PI;
            } else {
                self.phase_offsets[i] = 0.0;
            }
        }
    }

    pub fn step(&mut self, dt: f64, throttle: f64) -> ([f64; MAX_CYLINDERS], [f64; MAX_CYLINDERS]) {
        let r = self.crank_radius;
        let l = self.rod_length;
        let mut total_combustion_torque = 0.0;
        let mut exhaust_flows = [0.0; MAX_CYLINDERS];
        let mut intake_flows = [0.0; MAX_CYLINDERS];
        let mut total_inertia = self.base_crank_inertia * self.num_cylinders as f64 * self.displacement_scale;
        let mut rng = rand::thread_rng();

        // 20% vacuum at idle, 100% atmosphere at full throttle
        let intake_pressure = ATMOSPHERIC_PRESSURE * (0.2 + throttle * 0.8);
        
        let intake_flow_rate = 1.0 - (-dt / 0.002).exp();
        let exhaust_flow_rate = 1.0 - (-dt / 0.0005).exp();
        let rev_limit_rads = 8000.0 * (2.0 * PI / 60.0);

        for i in 0..self.num_cylinders {
            let theta = self.crank_angle + self.phase_offsets[i];
            let sin_t = theta.sin(); let cos_t = theta.cos();
            let rod_term = (l * l - r * r * sin_t * sin_t).sqrt();
            let dx_dtheta = -r * sin_t - ((r * r * sin_t * cos_t) / rod_term);

            let current_x = r * cos_t + rod_term;
            let volume = self.clearance_volume + self.piston_area * ((r + l) - current_x);

            let mut cycle_angle = theta % (4.0 * PI);
            if cycle_angle < 0.0 { cycle_angle += 4.0 * PI; }

            if let Some(prev_vol) = self.prev_volume[i] {
                self.cylinder_pressure[i] *= (prev_vol / volume).powf(1.4);
            }

            let is_intake = cycle_angle < 1.05 * PI || cycle_angle > 3.9 * PI; 
            let is_exhaust = cycle_angle > 2.9 * PI || cycle_angle < 0.1 * PI; 

            if is_intake {
                let diff = self.cylinder_pressure[i] - intake_pressure;
                let flow = diff * intake_flow_rate;
                self.cylinder_pressure[i] -= flow;

                let raw_suck = flow * 0.00001; 
                let turbulence = raw_suck.abs() * 0.8 * rng.gen_range(-1.0..1.0); 
                intake_flows[i] = raw_suck + turbulence;
            }

            if is_exhaust {
                let diff = self.cylinder_pressure[i] - ATMOSPHERIC_PRESSURE;
                let flow = diff * exhaust_flow_rate;
                self.cylinder_pressure[i] -= flow;

                let raw_pulse = flow * 0.00001;
                let turbulence = raw_pulse.abs() * 0.5 * rng.gen_range(-1.0..1.0);
                exhaust_flows[i] = raw_pulse + turbulence;
            }

            // COMBUSTION: Hard spark cut at 8000 RPM
            if self.prev_cycle_angle[i] < 2.0 * PI && cycle_angle >= 2.0 * PI {
                if self.angular_velocity < rev_limit_rads {
                    let jitter = rng.gen_range(0.95..1.05); 
                    // FIXED: Massive increase to combustion pressure multiplier (20x) so we hit the rev limiter
                    let boost = 1.0 + (20.0 * (0.1 + throttle * 0.9) * jitter);
                    self.cylinder_pressure[i] *= boost;
                }
            }

            self.prev_volume[i] = Some(volume);
            self.prev_cycle_angle[i] = cycle_angle;

            let net_pressure = self.cylinder_pressure[i] - ATMOSPHERIC_PRESSURE;
            let cylinder_pressure_force = net_pressure * self.piston_area;
            total_combustion_torque += -cylinder_pressure_force * dx_dtheta;
            total_inertia += self.piston_mass * (dx_dtheta * dx_dtheta);
        }

        // FIXED: Lowered Drag coefficients so the engine can rev out freely
        let friction_torque = -0.02 * self.num_cylinders as f64 * self.displacement_scale * self.angular_velocity;
        let aero_drag = -0.0001 * self.num_cylinders as f64 * self.displacement_scale * self.angular_velocity * self.angular_velocity.abs();

        let rpm_error = (1000.0 * 2.0 * PI / 60.0) - self.angular_velocity;
        let idle_torque = if rpm_error > 0.0 { rpm_error * 5.0 * self.num_cylinders as f64 * self.displacement_scale } else { 0.0 };

        let total_torque = total_combustion_torque + friction_torque + aero_drag + idle_torque;

        self.angular_velocity += (total_torque / total_inertia) * dt;
        if self.angular_velocity < 0.0 { self.angular_velocity = 0.0; } 
        self.crank_angle += self.angular_velocity * dt;
        if self.crank_angle > 2000.0 * PI { self.crank_angle -= 2000.0 * PI; }

        (exhaust_flows, intake_flows)
    }
}

// === 2. ACOUSTIC SIGNAL PROCESSING ===

struct ConvolutionFilter {
    ir: Vec<f64>,
    history: Vec<f64>,
    ptr: usize,
}

impl ConvolutionFilter {
    fn new(sample_rate: f64, resonance_freq: f64, decay_speed: f64, length: usize) -> Self {
        let mut ir = vec![0.0; length];
        let mut rng = rand::thread_rng();
        let mut lp = 0.0;

        for i in 0..length {
            let noise = rng.gen_range(-1.0..1.0);
            let envelope = (-(i as f64) / decay_speed).exp(); 
            let resonance = (i as f64 * 2.0 * PI * resonance_freq / sample_rate).cos(); 
            
            lp += 0.3 * ((noise * envelope * 0.5 + resonance * envelope * 0.5) - lp);
            ir[i] = lp;
        }

        Self { ir, history: vec![0.0; length], ptr: 0 }
    }

    fn process(&mut self, input: f64) -> f64 {
        self.history[self.ptr] = input;
        let mut sum = 0.0;
        let len = self.ir.len();
        for i in 0..len {
            let buf_idx = (self.ptr + len - i) % len;
            sum += self.history[buf_idx] * self.ir[i];
        }
        self.ptr = (self.ptr + 1) % len;
        sum
    }
}

struct HeaderDelay {
    buffer: Vec<f64>,
    ptr: usize,
}
impl HeaderDelay {
    fn new(delay_samples: usize) -> Self { Self { buffer: vec![0.0; delay_samples.max(1)], ptr: 0 } }
    fn process(&mut self, input: f64) -> f64 {
        let output = self.buffer[self.ptr];
        self.buffer[self.ptr] = input;
        self.ptr = (self.ptr + 1) % self.buffer.len();
        output
    }
}

struct AutoGain { peak: f64 }
impl AutoGain {
    fn new() -> Self { Self { peak: 1.0 } }
    fn process(&mut self, input: f64) -> f64 {
        let abs_i = input.abs();
        if abs_i > self.peak { self.peak = abs_i; } else { self.peak = self.peak * 0.999 + 1.0 * 0.001; }
        input / self.peak
    }
}

struct DcBlocker {
    x_prev: f64,
    y_prev: f64,
}
impl DcBlocker {
    fn new() -> Self { Self { x_prev: 0.0, y_prev: 0.0 } }
    fn process(&mut self, x: f64) -> f64 {
        let y = x - self.x_prev + 0.995 * self.y_prev;
        self.x_prev = x;
        self.y_prev = y;
        y
    }
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
    
    let mut agc = AutoGain::new();
    let mut dc_block = DcBlocker::new();
    let dt = 1.0 / sample_rate;

    let stream = device.build_output_stream(
        &config.into(),
        move |data: &mut [f32], _: &cpal::OutputCallbackInfo| {
            let target_cyls = state.cylinders.load(Ordering::Relaxed);
            let target_disp = f64::from_bits(state.displacement.load(Ordering::Relaxed));
            
            if target_cyls != engine.num_cylinders || target_disp != engine.displacement_scale || header_delays.is_empty() {
                engine.reconfigure(target_cyls, target_disp);
                header_delays.clear();
                
                for i in 0..target_cyls {
                    // FIXED: Group into banks (e.g. Left/Right) and add deterministic pseudo-random jitter.
                    // This absolutely destroys the 666Hz Comb Filter problem that makes V10s/V12s sound bad.
                    let bank_pos = (i / 2) as f64; 
                    let imperfection = ((i as f64) * 1.618).sin() * 0.4;
                    let delay_ms = 1.5 + (bank_pos * 1.2) + imperfection; 
                    
                    header_delays.push(HeaderDelay::new((delay_ms / 1000.0 * sample_rate) as usize));
                }
            }

            let throttle = f64::from_bits(state.throttle.load(Ordering::Relaxed));
            
            let mut last_ex = [0.0; MAX_CYLINDERS];
            let mut last_in = [0.0; MAX_CYLINDERS];

            for frame in data.chunks_mut(channels) {
                let (raw_exhausts, raw_intakes) = engine.step(dt, throttle);
                
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
                
                let drive = 3.0 + (throttle * 15.0); 
                let intake_vol = 0.2 + (throttle * 0.8); 
                
                let final_mix = ex_convolved + (in_convolved * intake_vol);
                let blocked = dc_block.process(final_mix);
                let overdriven = (blocked * drive).tanh() / drive.tanh();
                let normalized = agc.process(overdriven);
                
                let sample_f32 = (normalized * 0.8).clamp(-1.0, 1.0) as f32;
                for channel in frame.iter_mut() { *channel = sample_f32; }
            }

            state.rpm.store((engine.angular_velocity * (60.0 / (2.0 * PI))).to_bits(), Ordering::Relaxed);
            for i in 0..target_cyls {
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

    loop {
        let start_time = Instant::now();

        if event::poll(Duration::from_millis(5)).unwrap() {
            if let Event::Key(key) = event::read().unwrap() {
                match key.code {
                    KeyCode::Char('w') | KeyCode::Char('W') => target_throttle = 1.0,
                    KeyCode::Up => {
                        let mut c = state.cylinders.load(Ordering::Relaxed);
                        if c < MAX_CYLINDERS { c += 1; state.cylinders.store(c, Ordering::Relaxed); }
                    }
                    KeyCode::Down => {
                        let mut c = state.cylinders.load(Ordering::Relaxed);
                        if c > 1 { c -= 1; state.cylinders.store(c, Ordering::Relaxed); }
                    }
                    KeyCode::Right => {
                        let d = (f64::from_bits(state.displacement.load(Ordering::Relaxed)) * 1.1).min(10.0);
                        state.displacement.store(d.to_bits(), Ordering::Relaxed);
                    }
                    KeyCode::Left => {
                        let d = (f64::from_bits(state.displacement.load(Ordering::Relaxed)) / 1.1).max(0.1);
                        state.displacement.store(d.to_bits(), Ordering::Relaxed);
                    }
                    KeyCode::Char('q') | KeyCode::Esc => break,
                    KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => break,
                    _ => {}
                }
            }
        } else {
            target_throttle = 0.0;
        }

        actual_throttle += (target_throttle - actual_throttle) * 0.15;
        state.throttle.store(actual_throttle.to_bits(), Ordering::Relaxed);

        let rpm = f64::from_bits(state.rpm.load(Ordering::Relaxed));
        let num_cyls = state.cylinders.load(Ordering::Relaxed);
        let current_disp = f64::from_bits(state.displacement.load(Ordering::Relaxed));

        execute!(stdout, cursor::MoveTo(0, 0)).unwrap();
        write!(stdout, "🏎️   THE ANGE SYNTHESIZER 🏎️\r\n").unwrap();
        write!(stdout, "--- Realtime Aerodynamics + Audio Convolution ---\r\n\n").unwrap();
        write!(stdout, "[ W ]          : Rev Throttle\r\n").unwrap();
        write!(stdout, "[ Up/Down ]    : Cylinders     ({})\r\n", num_cyls).unwrap();
        write!(stdout, "[ Left/Right ] : Displacement  ({:.2}x)   \r\n", current_disp).unwrap();
        write!(stdout, "[ Q / ESC ]    : Quit\r\n\n").unwrap();

        let rpm_bar_len = ((rpm / 8000.0) * 40.0).clamp(0.0, 40.0) as usize;
        let rpm_bar = "█".repeat(rpm_bar_len) + &"-".repeat(40_usize.saturating_sub(rpm_bar_len));
        
        // Show REV LIMITER engaged text!
        if rpm > 7950.0 {
            write!(stdout, "RPM:      {:04.0} [|||||||||||||||| LIMITER |||||||||||||||]\r\n", rpm).unwrap();
        } else {
            write!(stdout, "RPM:      {:04.0} [{}]\r\n", rpm, rpm_bar).unwrap();
        }

        let t_bar_len = ((actual_throttle / 1.0) * 40.0).clamp(0.0, 40.0) as usize;
        let t_bar = "█".repeat(t_bar_len) + &"-".repeat(40_usize.saturating_sub(t_bar_len));
        write!(stdout, "Throttle: {:03.0}% [{}]\r\n\n", actual_throttle * 100.0, t_bar).unwrap();

        write!(stdout, "--- Realtime Cylinder Telemetry ---\r\n").unwrap();
        for i in 0..num_cyls {
            let p = f64::from_bits(state.pressures[i].load(Ordering::Relaxed));
            let inf = f64::from_bits(state.intake_flows[i].load(Ordering::Relaxed));
            let exf = f64::from_bits(state.exhaust_flows[i].load(Ordering::Relaxed));
            
            let p_kpa = p / 1000.0;
            
            write!(stdout, "Cyl {:02} | Press: {:7.1} kPa | Intake Flow: {:8.5} | Exhaust Flow: {:8.5}\r\n", 
                i + 1, p_kpa, inf, exf).unwrap();
        }
        write!(stdout, "{}", terminal::Clear(ClearType::FromCursorDown)).unwrap();

        stdout.flush().unwrap();
        let elapsed = start_time.elapsed();
        if elapsed < frame_duration { std::thread::sleep(frame_duration - elapsed); }
    }

    execute!(stdout, cursor::Show).unwrap();
    terminal::disable_raw_mode().unwrap();
}