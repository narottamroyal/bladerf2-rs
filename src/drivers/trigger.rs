//! Trigger control on J51-1.
//!
//! RX and TX each have a trigger register. While a direction is armed, its
//! samples flow only while the J51-1 line is low. The master drives the line
//! high as soon as it is made master, and low when it fires; slaves only read
//! it. The gate follows the line level: arming again makes the master drive
//! the line high and stops samples mid-stream. Arm before activating the
//! stream, so that the first samples after the fire are the first received.

use crate::channel::Direction;
use crate::error::{Error, Result};
use crate::nios::{Nios8x8, NiosCore};

/// Trigger register bits.
const ARM: u8 = 1 << 0;
const FIRE: u8 = 1 << 1;
const MASTER: u8 = 1 << 2;
/// Read-only level of the J51-1 line.
const LINE: u8 = 1 << 3;

/// A direction's part in the trigger.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerRole {
    /// Drives J51-1 and fires the trigger.
    Master,
    /// Waits for another device's master on J51-1.
    Slave,
    /// No trigger: the register is cleared.
    Disabled,
}

/// A direction's trigger state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TriggerState {
    /// The direction is armed, so its samples wait for the line.
    pub armed: bool,
    /// The line is low, so an armed direction's samples flow.
    pub fired: bool,
    /// This direction is the master and has been told to fire.
    pub fire_requested: bool,
}

fn target(direction: Direction) -> Nios8x8 {
    match direction {
        Direction::Rx => Nios8x8::RxTriggerCtl,
        Direction::Tx => Nios8x8::TxTriggerCtl,
    }
}

/// The register value after arming (`arm`) or disarming in `role`: any fire
/// request is cleared, and a disabled trigger clears the whole register.
fn armed(register: u8, role: TriggerRole, arm: bool) -> u8 {
    let register = register & !FIRE;
    let register = if arm { register | ARM } else { register & !ARM };
    match role {
        TriggerRole::Master => register | MASTER,
        TriggerRole::Slave => register & !MASTER,
        TriggerRole::Disabled => 0,
    }
}

fn state(register: u8) -> TriggerState {
    TriggerState {
        armed: register & ARM != 0,
        // The line is active low.
        fired: register & LINE == 0,
        fire_requested: register & (MASTER | FIRE) == MASTER | FIRE,
    }
}

/// Arm or disarm `direction`'s trigger in `role`. Fails with
/// [`Error::TriggerMaster`] if `role` is master and the other direction
/// already is.
pub(crate) async fn arm(
    nios: &NiosCore,
    direction: Direction,
    role: TriggerRole,
    on: bool,
) -> Result<()> {
    // Disarming as master also sets the master bit, so check that too.
    if role == TriggerRole::Master {
        let other = match direction {
            Direction::Rx => Direction::Tx,
            Direction::Tx => Direction::Rx,
        };
        if nios.read(target(other), 0).await? & MASTER != 0 {
            return Err(Error::TriggerMaster);
        }
    }
    let register = nios.read(target(direction), 0).await?;
    nios.write(target(direction), 0, armed(register, role, on))
        .await
}

/// Fire `direction`'s trigger. Only a master drives the line.
pub(crate) async fn fire(nios: &NiosCore, direction: Direction) -> Result<()> {
    let register = nios.read(target(direction), 0).await?;
    nios.write(target(direction), 0, register | FIRE).await
}

/// `direction`'s trigger state.
pub(crate) async fn trigger_state(nios: &NiosCore, direction: Direction) -> Result<TriggerState> {
    Ok(state(nios.read(target(direction), 0).await?))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arming_sets_the_register_bits() {
        assert_eq!(armed(0, TriggerRole::Master, true), ARM | MASTER);
        assert_eq!(
            armed(ARM | FIRE | MASTER, TriggerRole::Master, true),
            ARM | MASTER
        );
        assert_eq!(armed(ARM | MASTER, TriggerRole::Slave, true), ARM);
        assert_eq!(armed(ARM | MASTER, TriggerRole::Master, false), MASTER);
        assert_eq!(
            armed(ARM | FIRE | MASTER | LINE, TriggerRole::Disabled, true),
            0
        );
    }

    #[test]
    fn state_reads_active_low_line() {
        let waiting = state(ARM | MASTER | LINE);
        assert!(waiting.armed && !waiting.fired && !waiting.fire_requested);
        let fired = state(ARM | MASTER | FIRE);
        assert!(fired.armed && fired.fired && fired.fire_requested);
        // A slave's fire bit is not a request.
        assert!(!state(ARM | FIRE).fire_requested);
    }
}
