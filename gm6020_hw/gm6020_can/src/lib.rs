use std::f64::consts::PI;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use embedded_can::{Frame as EmbeddedFrame, StandardId};
use socketcan::{CanDataFrame, CanFilter, CanFrame, CanSocket, Frame, Socket, SocketOptions};
use core::slice;
use std::ptr::null;
use std::time::SystemTime;
use std::ffi::CStr;
use std::os::raw::c_char;
use std::thread::{self, JoinHandle};
use std::sync::Arc;


const FB_ID_BASE: u16 = 0x204;
const CMD_ID_V_L: u16 = 0x1ff;
const CMD_ID_V_H: u16 = 0x2ff;
const CMD_ID_I_L: u16 = 0x1fe;
const CMD_ID_I_H: u16 = 0x2fe;
pub const ID_MIN: u8 = 1;
pub const ID_MAX: u8 = 7;
const POS_MAX   : u16 = 8191;


const RPM_PER_V   : f64 =  13.33;
pub const N_PER_A : f64 = 741.0;
pub const V_MAX   : f64 =  24.0;
pub const I_MAX   : f64 =   1.62;
const I_FB_MAX    : f64 =   3.0;
const V_CMD_MAX : f64 = 25000.0;
const I_CMD_MAX : f64 = 16384.0;
const TEMP_MAX   : u8  = 125; // C

#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(C)]
pub enum CmdMode { Voltage, Current, Torque, Velocity }
impl Default for CmdMode {
    fn default() -> Self { CmdMode::Voltage }
}


#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(C)]
pub enum FbField { Position, Velocity, Current, Temperature }
impl Default for FbField {
    fn default() -> Self { FbField::Position }
}

#[derive(Copy, Clone, Debug)]
#[repr(C)]
enum IdRange { Low, High }

#[derive(Default, Debug)]
#[repr(C)]
struct Feedback {
    position:    u16, // [0, 8191]
    velocity:    i16, // rpm
    current:     i16, // [-16384, 16384]:[-3A, 3A]
    temperature: u16,  // TODO units
}

#[derive(Default)]
#[repr(C)]
pub struct Gm6020Can {
    socket: Option<CanSocket>,
    feedbacks: [(Option<SystemTime>, Feedback); 8], // (ID_MAX-ID_MIN+1) as usize   only 7 slots will be used but 8 is convenient for tx_cmd
    mode: CmdMode,
    commands: [i16; 8], // only 7 slots will be used but 8 is convenient for tx_cmd
}


// TODO split implementation and C wrapper into separate files
// TODO handle CAN no buffer left

#[no_mangle]
pub extern "C" fn gm6020_can_init(interface: *const c_char) -> *mut Gm6020Can{
    let inter: &str;
    if interface.is_null() {
        println!("Invalid c-string received for interface name (null pointer)");
        return null::<Gm6020Can>() as *mut Gm6020Can;
    }
    else {
        unsafe {
            let r = CStr::from_ptr(interface).to_str();
            if r.is_err() {
                eprintln!("Invalid c-string received for interface name");
                return null::<Gm6020Can>() as *mut Gm6020Can;
            }
            inter = r.unwrap();
        }
    }
    _init(inter).map_or_else(|e| {eprintln!("{}", e); null::<Gm6020Can>() as *mut Gm6020Can}, |v| Box::into_raw(v) as *mut Gm6020Can)
}
fn _init(interface: &str) -> Result<Box<Gm6020Can>, String> {
    let mut gm6020_can: Box<Gm6020Can> = Box::new(Gm6020Can::default()); // TODO technically a memory leak - also make sure socket is closed
    gm6020_can.as_mut().socket = Some(CanSocket::open(&interface).map_err(|err| err.to_string())?);
    let filter = CanFilter::new(FB_ID_BASE as u32, 0xffff - 0xf); // Only accept messages with IDs from 0x200 to 0x20F (Motor feedbacks are 0x205 to 0x20B)
    gm6020_can.as_ref().socket.as_ref().unwrap().set_filters(&[filter]).map_err(|err| err.to_string())?;
    return Ok(gm6020_can);
}

// Below 100Hz the feedback values get weird - CAN buffer filling up?
#[no_mangle]
pub extern "C" fn gm6020_can_run(gm6020_can: *mut Gm6020Can, period_ms: u64) -> i8{
    let handle: &mut Gm6020Can;
    if gm6020_can.is_null(){
        println!("Invalid handle (null pointer)");
        return -1;
    }
    else{
        handle = unsafe{&mut *gm6020_can};
    }
    // TODO Can we /should we use an async library instead
    thread::spawn( move || _run(handle, period_ms).map_or_else(|e| {eprintln!("{}", e); -1_i8}, |_| 0_i8));
    return 0;
}
fn _run(gm6020_can: &mut Gm6020Can, period_ms: u64) -> Result<(), String>{
    loop {
        _run_once(gm6020_can)?;
        thread::sleep(std::time::Duration::from_millis(period_ms));
    }
}
#[no_mangle]
pub extern "C" fn gm6020_can_run_once(gm6020_can: *mut Gm6020Can) -> i8{
    let handle: &mut Gm6020Can;
    if gm6020_can.is_null(){
        println!("Invalid handle (null pointer)");
        return -1;
    }
    else{
        handle = unsafe{&mut *gm6020_can};
    }
    _run_once(handle).map_or_else(|e| {eprintln!("{}", e); -1_i8}, |_| 0_i8)
}
fn _run_once(gm6020_can: &mut Gm6020Can) -> Result<(), String>{
    match gm6020_can.socket.as_ref().unwrap().read_frame_timeout(Duration::from_millis(2)) { // Feedbacks sent at 1kHz, use 2ms for slight leeway
        Ok(CanFrame::Data(frame)) => rx_fb(gm6020_can, frame),
        Ok(CanFrame::Remote(frame)) => eprintln!("{:?}", frame),
        Ok(CanFrame::Error(frame)) => eprintln!("{:?}", frame),
        Err(err) => eprintln!("{}", err),
    };

    tx_cmd(gm6020_can, IdRange::Low).and_then(|_| tx_cmd(gm6020_can, IdRange::High))
}

fn set_cmd(gm6020_can: &mut Gm6020Can, id: u8, mode: CmdMode, cmd: f64) -> Result<(), String> {
    let idx = (id-1) as usize;
    if gm6020_can.feedbacks[idx].1.temperature >= TEMP_MAX as u16 { gm6020_can.commands[idx] = 0; return Err(format!("temperature overload [{}]: {}", TEMP_MAX, gm6020_can.feedbacks[idx].1.temperature));}
    if mode == CmdMode::Torque {return set_cmd(gm6020_can, id, CmdMode::Current, cmd/N_PER_A);}
    if mode == CmdMode::Velocity  {return set_cmd(gm6020_can, id, CmdMode::Voltage, cmd/RPM_PER_V);}
    if mode == CmdMode::Voltage && cmd.abs() > V_MAX { return Err(format!("voltage out of range [{}, {}]: {}", -1.0*V_MAX, V_MAX, cmd));}
    if mode == CmdMode::Current && cmd.abs() > I_MAX { return Err(format!("current out of range [{}, {}]: {}", -1.0*I_MAX, I_MAX, cmd));}
    if mode != gm6020_can.mode {
        println!("Warning: changing command mode affects all motors on this bus.");
        for i in 0..(ID_MAX-ID_MIN)as usize{
            gm6020_can.commands[i] = 0;
        }
    }
    gm6020_can.commands[idx] = match mode {
        CmdMode::Voltage => (V_CMD_MAX/V_MAX*cmd) as i16,
        CmdMode::Current => (I_CMD_MAX/I_MAX*cmd) as i16,
        _ => panic!(),
    };
    Ok(())
}


#[no_mangle]
pub extern "C" fn gm6020_can_cmd_single(gm6020_can: *mut Gm6020Can, mode: CmdMode, id: u8, cmd: f64) -> i8{
    let handle: &mut Gm6020Can;
    if gm6020_can.is_null(){
        println!("Invalid handle (null pointer)");
        return -1;
    }
    else{
        handle = unsafe{&mut *gm6020_can};
    }
    _cmd_single(handle, mode, id, cmd).map_or_else(|e| {eprintln!("{}", e); -1_i8}, |_| 0_i8)
}
fn _cmd_single(gm6020_can: &mut Gm6020Can, mode: CmdMode, id: u8, cmd: f64) -> Result<(), String> {
    if id<ID_MIN || id>ID_MAX { return Err(format!("id out of range [{}, {}]: {}", ID_MIN, ID_MAX, id)); }
    set_cmd(gm6020_can, id, mode, cmd)?;
    Ok(())
}

#[no_mangle]
pub extern "C" fn gm6020_can_cmd_multiple(gm6020_can: *mut Gm6020Can, mode: CmdMode, cmds: *const *const f64, len: u8) -> i8{
    let handle: &mut Gm6020Can;
    let cmds2: &[&[f64]]; // TODO better naming
    if gm6020_can.is_null() || cmds.is_null(){
        println!("Invalid handle or commands (null pointer)");
        return -1;
    }
    else{
        handle = unsafe{&mut *gm6020_can};
        cmds2 = unsafe {slice::from_raw_parts(cmds as *const &[f64], len as usize)} // TODO how can we assert valid data for this?
    }
    let mut cmds3: Vec<(u8, f64)> = Vec::new();
    for i in 0..(len as usize) {
        cmds3.push((cmds2[i][0 as usize] as u8, cmds2[i][1 as usize]));
    }
    _cmd_multiple(handle, mode, cmds3).map_or_else(|e| {eprintln!("{}", e); -1_i8}, |_| 0_i8)
}
fn _cmd_multiple(gm6020_can: &mut Gm6020Can, mode: CmdMode, cmds: Vec<(u8, f64)> ) -> Result<(), String> {
    for cmd in cmds.into_iter(){
        set_cmd(gm6020_can, cmd.0, mode, cmd.1)?;
    }
    Ok(())
}

fn tx_cmd(gm6020_can: &mut Gm6020Can, id_range: IdRange) -> Result<(), String> {
    // TODO change this to a warning, don't panic
    for (i, fb) in (&gm6020_can.feedbacks[((id_range as u8) * 4) as usize .. (4 + (id_range as u8)*4) as usize]).iter().enumerate() {
        if gm6020_can.commands[(i as u8 + (id_range as u8)*4) as usize] != 0 && fb.0.ok_or_else(|| Err::<(), String>(format!("Motor {} never responded. Did you enter the `run` loop?", (i as u8)+ID_MIN))).unwrap().elapsed().map_err(|err| err.to_string())?.as_millis() >= 10 {
            return Err(format!("Motor {} not responding. Did you enter the `run` loop?", (i as u8)+ID_MIN));
        }
    }

    let id: u16 = match (id_range, gm6020_can.mode) {
        (IdRange::Low,  CmdMode::Voltage) => CMD_ID_V_L,
        (IdRange::High, CmdMode::Voltage) => CMD_ID_V_H,
        (IdRange::Low,  CmdMode::Current) => CMD_ID_I_L,
        (IdRange::High, CmdMode::Current) => CMD_ID_I_H,
        (_, _) => panic!(),
    };
    let cmds: &[i16] = &gm6020_can.commands[((id_range as u8) * 4) as usize .. (4 + (id_range as u8)*4) as usize];
    // TODO can we set byte alignment of commands so this can be written directly? Might need to check endian-ness
    let frame = CanFrame::new(
        StandardId::new(id).unwrap(),
        &[(cmds[0]>>8) as u8, cmds[0] as u8, (cmds[1]>>8) as u8, cmds[1] as u8, (cmds[2]>>8) as u8, cmds[2] as u8, (cmds[3]>>8) as u8, cmds[3] as u8])
        .ok_or_else(|| Err::<CanFrame, String>("Failed to open socket".to_string())).unwrap();
    gm6020_can.socket.as_ref().ok_or_else(|| Err::<CanSocket, String>("Socket not initialized".to_string())).unwrap().write_frame(&frame).map_err(|err| err.to_string())?;
    Ok(())
}

fn rx_fb(gm6020_can: &mut Gm6020Can, frame: CanDataFrame){
    let rxid: u16 = frame.raw_id() as u16;
    let id: u8 = (rxid-FB_ID_BASE)as u8;
    if id<ID_MIN || id>ID_MAX {return;}
    let idx: usize = (id-1) as usize;
    let f: &mut (Option<SystemTime>, Feedback) = &mut gm6020_can.feedbacks[idx];
    let d: &[u8] = &frame.data()[0..8];
    f.0 = Some(SystemTime::now());// TODO waiting on socketcan library to implement hardware timestamps
    f.1.position    = (d[0] as u16) << 8 | d[1] as u16;
    f.1.velocity    = (d[2] as i16) << 8 | d[3] as i16;
    f.1.current     = (d[4] as i16) << 8 | d[5] as i16;
    f.1.temperature = d[6] as u16;
    // Apparently this is frowned-upon but it looks a lot cooler
    //unsafe {
    //    f.1 = std::mem::transmute::<[u8; 8], Feedback>(frame.data()[0..8].try_into().unwrap());
    //}
    //f.1.temperature = f.1.temperature >> 8; // TODO check endianness
}

#[no_mangle]
pub extern "C" fn gm6020_can_get(gm6020_can: *mut Gm6020Can, id: u8, field: FbField) -> f64{
    if id<ID_MIN || id>ID_MAX { eprintln!("id out of range [{}, {}]: {}", ID_MIN, ID_MAX, id); panic!();}
    let handle: &mut Gm6020Can;
    if gm6020_can.is_null(){
        println!("Invalid handle");
        panic!();
    }
    else{
        handle = unsafe{&mut *gm6020_can};
    }
    match field {
        FbField::Position    => handle.feedbacks[(id-1)as usize].1.position as f64/POS_MAX as f64 *2f64*PI,
        FbField::Velocity    => handle.feedbacks[(id-1)as usize].1.velocity as f64,
        FbField::Current     => handle.feedbacks[(id-1)as usize].1.current as f64 / 1000f64,
        FbField::Temperature => handle.feedbacks[(id-1)as usize].1.temperature as f64,
    }
}


pub fn debug_thread(gm6020_can: *mut Gm6020Can, id: u8, field: FbField, stop: Arc<AtomicBool>) -> JoinHandle<()>{
    let handle: &mut Gm6020Can;
    if gm6020_can.is_null(){
        println!("Invalid handle (null pointer)");
        panic!();
    }
    else{
        handle = unsafe{&mut *gm6020_can};
    }
    thread::spawn( move ||
        while ! stop.load(Ordering::Relaxed){
            thread::sleep(std::time::Duration::from_millis(50));
            let val = gm6020_can_get(handle, id, field);
            print!("{}\t", val);
            match field {
                FbField::Current => println!("{:#<1$}", "", (val.abs()/I_FB_MAX*20f64) as usize),
                FbField::Temperature => println!("{:#<1$}", "", val as usize),
                _ => println!("")
            }
        }
    )
}