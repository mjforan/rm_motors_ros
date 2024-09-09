use gm6020_can::{CmdMode, FbField, Gm6020Can};
use std::ffi::CString;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::sync::Arc;

//////
// Basic example showing how to use gm6020_can library.
//////
/*
cargo build --release --examples && ./target/release/examples/gm6020_can_test
*/

const RATE: u64 = 100; // Should be above 100Hz. At slower rates the feedback values get weird and eventually the commands stop going out. Could be hardware-dependent.
const PERIOD: u64 = (1.0f64/(RATE as f64)*1000.0)as u64;
const INC: u64 = 10;                              // Time between commands
const MAX: i16 = (gm6020_can::V_MAX)as i16 * 10;  // Need the 10x multiplier so we can easily increment in for loops (can't increment floats).
const ID: u8 = 1;                                 // Motor ID [1,7]

fn main() {
    // Open SocketCAN device
    let ifname: std::ffi::CString = CString::new("can0").expect("CString::new failed");
    let gmc_: *mut Gm6020Can = gm6020_can::gm6020_can_init(ifname.as_ptr());
    if gmc_.is_null(){
        println!("Unable to open specified SocketCAN device");
        return;
    }

    // Start another thread to collect feedback values
    gm6020_can::gm6020_can_run(gmc_, PERIOD);

    // Start another thread to print current values
    let shared_stop = Arc::new(AtomicBool::new(false)).clone();
    let dbg = gm6020_can::debug_thread(gmc_, ID, FbField::Current, shared_stop.clone());

    // Ramp up, ramp down, ramp up (negative), ramp down (negative)
    for voltage in (0 .. MAX+1).step_by(2) {
        gm6020_can::gm6020_can_cmd_single(gmc_, CmdMode::Voltage, ID, voltage as f64 / 10f64);
        thread::sleep(std::time::Duration::from_millis(INC));
    }
    for voltage in (0 .. MAX).rev().step_by(2) {
        gm6020_can::gm6020_can_cmd_single(gmc_, CmdMode::Voltage, ID, voltage as f64 / 10f64);
        thread::sleep(std::time::Duration::from_millis(INC));
    }
    for voltage in (-1*MAX .. 0).rev().step_by(2) {
        gm6020_can::gm6020_can_cmd_single(gmc_, CmdMode::Voltage, ID, voltage as f64 / 10f64);
        thread::sleep(std::time::Duration::from_millis(INC));
    }
    for voltage in (-1*MAX+1 .. 1).step_by(2) {
        gm6020_can::gm6020_can_cmd_single(gmc_, CmdMode::Voltage, ID, voltage as f64 / 10f64);
        thread::sleep(std::time::Duration::from_millis(INC));
    }
    // Stop the thread that was printing current values
    shared_stop.store(true, Ordering::Relaxed);
    let _ = dbg.join();

    // Send constant voltage command and read out position feedback
    gm6020_can::gm6020_can_cmd_single(gmc_, CmdMode::Voltage, ID, 1f64);
    loop{
        thread::sleep(std::time::Duration::from_millis(50));
        println!("{}", gm6020_can::gm6020_can_get(gmc_, ID, FbField::Position));
    }
}

